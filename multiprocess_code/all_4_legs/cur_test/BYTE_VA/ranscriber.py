# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/ranscriber.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Offline speech-to-text with faster-whisper (tiny.en, CPU, int8),
# biased by an initial prompt toward short robot commands.
# NOTE      ·  File name is a typo for 'transcriber.py'; app.py imports it
#              under this name, so rename both together.
# ════════════════════════════════════════════════════════════════════════
"""
transcriber.py
--------------
Offline speech-to-text using faster-whisper (CTranslate2 backend).

Model  : tiny.en  (English-only, ~39 MB, runs comfortably on CPU)
Device : cpu
Compute: int8  (fastest CPU inference, negligible accuracy loss)

The initial_prompt biases Whisper's decoder toward short robot-dog
commands so it prefers "sit" over "set", "fetch" over "fetch it", etc.
"""

from pathlib import Path

import numpy as np
from faster_whisper import WhisperModel

SAMPLE_RATE = 16_000
_INITIAL_PROMPT = "commands like sit, stay, walk, fetch, come, heel, down, spin"


class Transcriber:
    """
    Wraps a faster-whisper WhisperModel and exposes a single
    ``transcribe(audio_np) -> str`` method.

    Parameters
    ----------
    model_size   : Whisper model identifier ("tiny.en" by default).
                   faster-whisper downloads the model on first use and
                   caches it under ~/.cache/huggingface/hub/.
    device       : "cpu" or "cuda"
    compute_type : "int8" (default) | "float16" | "float32"
    initial_prompt : Decoding hint that biases the model toward short
                     robot-dog commands.
    """

    def __init__(
        self,
        model_size: str = "base.en",
        device: str = "cpu",
        compute_type: str = "int8",
        initial_prompt: str = _INITIAL_PROMPT,
    ) -> None:
        print(f"[Transcriber] Loading faster-whisper '{model_size}' …", flush=True)
        self._model = WhisperModel(
            model_size,
            device=device,
            compute_type=compute_type,
        )
        self._initial_prompt = initial_prompt
        print("[Transcriber] Model ready.", flush=True)

    # ---------------------------------------------------------------------- #
    # Public API
    # ---------------------------------------------------------------------- #
    def transcribe(self, audio_np: np.ndarray) -> str:
        """
        Transcribe a float32 16 kHz mono audio array.

        Parameters
        ----------
        audio_np : np.ndarray
            Shape (N,), dtype float32, values in [-1.0, 1.0].

        Returns
        -------
        str
            Stripped, lower-cased transcription text (empty string on silence).
        """
        if audio_np.size == 0:
            return ""

        segments, _info = self._model.transcribe(
            audio_np,
            language="en",
            initial_prompt=self._initial_prompt,
            beam_size=3,          # fastest; use 5 for better accuracy
            vad_filter=True,      # skip internal silent segments
            vad_parameters={
                "min_silence_duration_ms": 300,
            },
        )

        parts = [seg.text.strip() for seg in segments]
        return " ".join(parts).strip().lower()