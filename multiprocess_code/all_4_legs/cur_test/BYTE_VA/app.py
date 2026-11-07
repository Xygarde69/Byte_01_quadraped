# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/app.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Voice-assistant main loop: LISTENING (VAD + wake-word) -> RECORDING (3
# s) -> TRANSCRIBING (faster-whisper). If the text contains 'stand' or
# 'sit' it runs byte_stand.py / byte_sit.py as a subprocess.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 app.py   (inside BYTE_VA/, with its byte_env)
# NOTE      ·  byte_stand.py / byte_sit.py use the OLD absolute-coordinate
#              protocol; with the robot/ runtime they would be interpreted
#              as offsets from SIT_COORDS (wrong or unreachable pose).
# ════════════════════════════════════════════════════════════════════════

import sys
import subprocess 
import time
from enum import Enum, auto

import numpy as np

from audio_stream import AudioStream
from ranscriber import Transcriber
from wake_word_engine import WakeWordEngine

# --------------------------------------------------------------------------- #
# Tunable constants
# --------------------------------------------------------------------------- #
SAMPLE_RATE          = 16_000
VAD_CHUNK_SIZE       = 4096        # samples per VAD tick (~128 ms); larger = KWS keeps up with normal-speed speech
COMMAND_DURATION_SEC = 3         # seconds to record after wake word
COMMAND_SAMPLES      = int(SAMPLE_RATE * COMMAND_DURATION_SEC)

# --------------------------------------------------------------------------- #
# State machine
# --------------------------------------------------------------------------- #
class State(Enum):
    LISTENING    = auto()
    RECORDING    = auto()
    TRANSCRIBING = auto()


def main() -> None:
    print("=" * 60)
    print("  BYTE – Robot Dog Voice Assistant")
    print("=" * 60)

    print("[BYTE] Initialising audio stream …", flush=True)
    audio = AudioStream(sample_rate=SAMPLE_RATE)

    print("[BYTE] Loading wake-word engine …", flush=True)
    engine = WakeWordEngine()

    print("[BYTE] Loading transcriber …", flush=True)
    transcriber = Transcriber()

    audio.start()
    print()
    print("[BYTE] Listening … Say 'Hey Byte' or 'Hello Byte' to activate.")
    print("       Press Ctrl+C to quit.\n")

    state = State.LISTENING
    recording_start: float = 0.0

    try:
        while True:
            # ---------------------------------------------------------------- #
            # LISTENING  – feed small chunks to VAD + KWS                      #
            # ---------------------------------------------------------------- #
            if state is State.LISTENING:
                if audio.available() < VAD_CHUNK_SIZE:
                    time.sleep(0.01)
                    continue

                chunk = audio.drain(VAD_CHUNK_SIZE)

                chunk = chunk * 1.5  # Amplify the quiet Windows mic by 8x
                chunk = np.clip(chunk, -1.0, 1.0) # Prevent audio distortion
                
                max_vol = np.max(np.abs(chunk))
                if max_vol > 0.1: # Only print when it's somewhat loud
                    print(f"[Debug] Vol: {max_vol:.2f}", flush=True)

                if engine.process(chunk):
                    kw = engine.last_keyword
                    # Hide internal phonetic 'bite' spelling for the demo
                    kw_display = kw.replace("bite", "byte").replace("Bite", "byte").title()
                    # e.g. 'hello bite' -> 'hello byte' -> 'Hello Byte'
                    print(f"[BYTE] '{kw_display}' detected! Listening for command …",
                          flush=True)
                    audio.clear()           # discard audio captured before trigger
                    engine.reset()
                    recording_start = time.monotonic()
                    state = State.RECORDING

            # ---------------------------------------------------------------- #
            # RECORDING  – accumulate COMMAND_DURATION_SEC of audio            #
            # ---------------------------------------------------------------- #
            elif state is State.RECORDING:
                elapsed = time.monotonic() - recording_start
                if elapsed >= COMMAND_DURATION_SEC:
                    state = State.TRANSCRIBING
                else:
                    # small sleep to avoid busy-spinning while audio fills up
                    time.sleep(0.02)

            # ---------------------------------------------------------------- #
            # TRANSCRIBING  – pull buffered audio, run Whisper                 #
            # ---------------------------------------------------------------- #
            elif state is State.TRANSCRIBING:
                command_audio = audio.read_chunk(COMMAND_SAMPLES)

                if command_audio.size == 0:
                    print("[BYTE] No audio captured – returning to LISTENING.",
                          flush=True)
                else:
                    print("[BYTE] Transcribing …", flush=True)
                    result = transcriber.transcribe(command_audio)

                    if result:
                        print(f"\n[BYTE] Command: {result}\n", flush=True)
                        cmd = result.lower()

                        if "stand" in cmd:
                            print("[BYTE] >> Logic Match! Executing STAND protocol...", flush=True)
                            try:
                                subprocess.run([sys.executable, "byte_stand.py"], check=True)
                            except Exception as e:
                                print(f"[BRIDGE ERROR] Could not trigger stand: {e}", flush=True)
                    
                        elif "sit" in cmd:
                            print("[BYTE] >> Logic Match! Executing SIT protocol...", flush=True)
                            try:
                                subprocess.run([sys.executable, "byte_sit.py"], check=True)
                            except Exception as e:
                                print(f"[BRIDGE ERROR] Could not trigger sit: {e}", flush=True)
                        else:
                            print("[BYTE] (no command heard)\n", flush=True)

                # Always clear and go back to listening
                audio.clear()
                engine.reset()
                print("[BYTE] Listening … Say 'Hey Byte' or 'Hello Byte' to activate.")
                state = State.LISTENING

    except KeyboardInterrupt:
        print("\n[BYTE] Shutting down …")
    finally:
        audio.stop()
        print("[BYTE] Goodbye.")


if __name__ == "__main__":
    main()