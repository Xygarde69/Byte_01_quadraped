# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/audio_stream.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Non-blocking 16 kHz mono microphone capture (sounddevice) into a
# thread-safe ring buffer of float32 samples.
# ════════════════════════════════════════════════════════════════════════
"""
audio_stream.py
---------------
Non-blocking 16 kHz mono audio capture using sounddevice.
Audio samples are stored in a thread-safe deque (ring buffer) as
float32 values in the range [-1.0, 1.0].
"""

import collections
import threading

import numpy as np
import sounddevice as sd

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
SAMPLE_RATE = 16_000          # Hz  – sherpa-onnx & Whisper both need 16 kHz
CHANNELS = 4                  # Mono
DTYPE = "float32"             # sounddevice native float; no int16 conversion needed
BLOCK_SIZE = 512              # Samples per callback (~32 ms @ 16 kHz)
RING_BUFFER_SECONDS = 30      # How many seconds of audio to keep in the ring buffer


class AudioStream:
    """
    Captures audio from the default input device into a ring buffer.

    Usage
    -----
    stream = AudioStream()
    stream.start()
    ...
    chunk = stream.read_chunk(8000)   # read 0.5 s of audio
    stream.clear()
    stream.stop()
    """

    def __init__(
        self,
        sample_rate: int = SAMPLE_RATE,
        block_size: int = BLOCK_SIZE,
        ring_buffer_seconds: float = RING_BUFFER_SECONDS,
        device=0,  
    ) -> None:
        self.sample_rate = sample_rate
        self.block_size = block_size
        self._device = device

        max_samples = int(ring_buffer_seconds * sample_rate)
        self._buffer: collections.deque[float] = collections.deque(maxlen=max_samples)
        self._lock = threading.Lock()


        self._stream = sd.InputStream(
            samplerate=sample_rate,
            channels=6,
            dtype=DTYPE,
            blocksize=block_size,
            device=self._device,
            callback=self._callback,
        )

    # ---------------------------------------------------------------------- #
    # sounddevice callback  (runs in a background C thread)
    # ---------------------------------------------------------------------- #
    def _callback(
        self,
        indata: np.ndarray,
        frames: int,
        time,
        status: sd.CallbackFlags,
    ) -> None:
        if status:
            # Print warnings (e.g. input overflow) without crashing
            print(f"[AudioStream] Warning: {status}", flush=True)
            mono_data = indata[:, 0]
        samples = indata[:, 0]          # flatten to 1-D
        rms = np.sqrt(np.mean(samples**2))
        # if rms > 0.05:  # Arbitrary threshold to log loud noises
        #     print(f"[Audio] Loud sound detected! RMS={rms:.4f}", flush=True)

        with self._lock:
            self._buffer.extend(samples.tolist())

    # ---------------------------------------------------------------------- #
    # Public API
    # ---------------------------------------------------------------------- #
    def start(self) -> None:
        """Start capturing audio."""
        self._stream.start()

    def stop(self) -> None:
        """Stop capturing audio and release the device."""
        self._stream.stop()
        self._stream.close()

    def available(self) -> int:
        """Return the number of samples currently in the ring buffer."""
        with self._lock:
            return len(self._buffer)

    def read_chunk(self, n_samples: int) -> np.ndarray:
        """
        Return up to *n_samples* samples from the front of the ring buffer
        as a float32 NumPy array.  Does NOT remove samples from the buffer;
        call clear() explicitly when you want to drain it.
        """
        with self._lock:
            data = list(self._buffer)
        data = data[-n_samples:] if len(data) > n_samples else data
        return np.array(data, dtype=np.float32)

    def drain(self, n_samples: int) -> np.ndarray:
        """
        Pop and return up to *n_samples* samples (oldest first) from the
        ring buffer, removing them in the process.
        """
        with self._lock:
            out = []
            for _ in range(min(n_samples, len(self._buffer))):
                out.append(self._buffer.popleft())
        return np.array(out, dtype=np.float32)

    def read_all(self) -> np.ndarray:
        """Return a copy of the entire ring buffer without clearing it."""
        with self._lock:
            return np.array(list(self._buffer), dtype=np.float32)

    def clear(self) -> None:
        """Discard all buffered audio."""
        with self._lock:
            self._buffer.clear()

    # ---------------------------------------------------------------------- #
    # Context-manager support
    # ---------------------------------------------------------------------- #
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()