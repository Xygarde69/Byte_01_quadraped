# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/wake_word_engine.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Two-stage wake word: Silero VAD (sherpa-onnx) gates audio, then a
# Zipformer keyword spotter looks for the keywords. Includes a Windows
# DLL-path workaround for onnxruntime.
# ════════════════════════════════════════════════════════════════════════
"""
wake_word_engine.py
-------------------
Phase 1 – Voice Activity Detection  (sherpa-onnx Silero VAD)
Phase 2 – Keyword Spotting          (sherpa-onnx Zipformer KWS)

Windows DLL fix
~~~~~~~~~~~~~~~
On Windows, Python's DLL search order can pick up a stale onnxruntime.dll
from System32 before the one bundled with sherpa-onnx, causing an API
version mismatch at import time.  We call os.add_dll_directory() pointing
at sherpa-onnx's own package directory *before* the first import of
sherpa_onnx to ensure the correct DLL is loaded.
"""

import os
import sys
from pathlib import Path

import numpy as np

# ------------------------------------------------------------------ #
# Windows DLL search-path fix  (must happen before "import sherpa_onnx")
# ------------------------------------------------------------------ #
def _fix_windows_dll_path() -> None:
    """Add the sherpa_onnx package directory to the DLL search path."""
    if sys.platform != "win32":
        return
    try:
        import importlib.util
        spec = importlib.util.find_spec("sherpa_onnx")
        if spec and spec.origin:
            pkg_dir = str(Path(spec.origin).parent)
            os.add_dll_directory(pkg_dir)
    except Exception as exc:  # noqa: BLE001
        print(f"[WakeWordEngine] DLL path fix skipped: {exc}", flush=True)


_fix_windows_dll_path()

import sherpa_onnx

_ROOT = Path(__file__).parent
_MODEL_DIR = _ROOT / "models" / "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01"

DEFAULT_VAD_MODEL     = str(_ROOT / "models" / "silero_vad.onnx")
DEFAULT_ENCODER       = str(_MODEL_DIR / "encoder-epoch-12-avg-2-chunk-16-left-64.onnx")
DEFAULT_DECODER       = str(_MODEL_DIR / "decoder-epoch-12-avg-2-chunk-16-left-64.onnx")
DEFAULT_JOINER        = str(_MODEL_DIR / "joiner-epoch-12-avg-2-chunk-16-left-64.onnx")
DEFAULT_TOKENS        = str(_MODEL_DIR / "tokens.txt")
DEFAULT_KEYWORDS_FILE = str(_ROOT / "keywords.txt")

SAMPLE_RATE = 16_000

# Set True to log raw KWS result when VAD goes inactive (to debug missed detections)
DEBUG_KWS_RESULT = True


class WakeWordEngine:
    """
    Combines Silero VAD + Zipformer Keyword Spotter to detect multi-word
    wake phrases ("Hey Bite" / "Hello Bite").

    Parameters
    ----------
    vad_model     : path to silero_vad.onnx
    encoder       : path to Zipformer encoder .onnx
    decoder       : path to Zipformer decoder .onnx
    joiner        : path to Zipformer joiner  .onnx
    tokens        : path to tokens.txt
    keywords_file : path to tokenized keywords.txt  (produced by the CLI)
    num_trailing_blanks : KWS tuning – blanks required after keyword
    """

    def __init__(
        self,
        vad_model: str = DEFAULT_VAD_MODEL,
        encoder: str = DEFAULT_ENCODER,
        decoder: str = DEFAULT_DECODER,
        joiner: str = DEFAULT_JOINER,
        tokens: str = DEFAULT_TOKENS,
        keywords_file: str = DEFAULT_KEYWORDS_FILE,
        num_trailing_blanks: int = 2,
    ) -> None:
        # ---- VAD --------------------------------------------------------- #
        vad_config = sherpa_onnx.VadModelConfig(
            silero_vad=sherpa_onnx.SileroVadModelConfig(
                model=vad_model,
                threshold=0.5,
                min_silence_duration=1.0,
                min_speech_duration=0.1,
            ),
            sample_rate=SAMPLE_RATE,
        )
        self._vad = sherpa_onnx.VoiceActivityDetector(
            vad_config,
            buffer_size_in_seconds=30,
        )

        # ---- Keyword Spotter --------------------------------------------- #
        # sherpa_onnx.KeywordSpotter takes flat positional/keyword args;
        # there is no KeywordSpotterConfig wrapper in this version.
        self._kws = sherpa_onnx.KeywordSpotter(
            tokens=tokens,
            encoder=encoder,
            decoder=decoder,
            joiner=joiner,
            keywords_file=keywords_file,
            num_threads=2,
            sample_rate=SAMPLE_RATE,  # must be int
            feature_dim=80,
            keywords_score=2.3,        # stronger boost so soft "B" in Bite/Byte still triggers
            keywords_threshold=0.002,  # lower = easier to trigger (helps with less emphatic B)
            max_active_paths=4,
            num_trailing_blanks=num_trailing_blanks,
            provider="cpu",
        )
        self._stream = self._kws.create_stream()

        # Track the last detected keyword for use in log messages
        self.last_keyword: str = ""


    # ---------------------------------------------------------------------- #
    # Public API
    # ---------------------------------------------------------------------- #
    def process(self, audio_chunk: np.ndarray) -> bool:
        """
        Feed *audio_chunk* (float32, 16 kHz) through VAD, then through the
        keyword spotter if speech is active.

        Returns True if a wake word was detected in this chunk.
        """
        # --- VAD gate ------------------------------------------------------ #
        self._vad.accept_waveform(audio_chunk)
        speech_detected = self._vad.is_speech_detected()

        if speech_detected and not getattr(self, "_was_speech", False):
            print("[VAD] Active", flush=True)
            self._was_speech = True
        elif not speech_detected and getattr(self, "_was_speech", False):
            print("[VAD] Inactive", flush=True)
            self._was_speech = False
            # Do NOT reset KWS stream here: the decoder may still be about to
            # emit the keyword (e.g. after "Hey Bite" with a short trailing
            # silence). Resetting here was preventing wake-word detection.
            # Stream is reset on actual keyword detection and in reset().
            if DEBUG_KWS_RESULT and hasattr(self, "_last_kws_result"):
                r = self._last_kws_result
                
                # Format string to hide the internal 'bite' spelling
                val_str = str(r).replace("bite", "byte").replace("Bite", "byte").title()
                
                print(
                    f"[KWS] Last result: type={type(r).__name__!r} value={val_str!r}",
                    flush=True,
                )

        # (We no longer return False here; KWS gets continuous audio context)

        # --- Keyword Spotter ----------------------------------------------- #
        self._stream.accept_waveform(SAMPLE_RATE, audio_chunk)
        while self._kws.is_ready(self._stream):
            self._kws.decode_stream(self._stream)

        result = self._kws.get_result(self._stream)
        if DEBUG_KWS_RESULT:
            self._last_kws_result = result

        # API returns str (keyword text or empty); some builds return an object with .keyword
        if result is None:
            return False
        if isinstance(result, str):
            kw_str = result.strip()
        else:
            kw_str = (
                getattr(result, "keyword", None) or getattr(result, "keyword_str", None)
            )
            kw_str = (kw_str or "").strip() if kw_str else ""
        if kw_str:
            self.last_keyword = kw_str.strip()
            self._kws.reset_stream(self._stream)
            return True

        return False

    def reset(self) -> None:
        """Reset both VAD and KWS state (call after each command cycle)."""
        self._vad.reset()
        self._kws.reset_stream(self._stream)
        self.last_keyword = ""