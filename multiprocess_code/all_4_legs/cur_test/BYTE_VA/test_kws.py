# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/test_kws.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Quick test: WakeWordEngine fed from AudioStream, prints when the wake
# word fires.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 test_kws.py
# ════════════════════════════════════════════════════════════════════════
import time
import numpy as np
from audio_stream import AudioStream
from wake_word_engine import WakeWordEngine

def main():
    engine = WakeWordEngine()
    audio = AudioStream(sample_rate=16000)
    audio.start()

    print("Listening for wake word (TEST SCRIPT)...")
    try:
        while True:
            if audio.available() >= 512:
                chunk = audio.drain(512)
                chunk = chunk * 8.0 
                chunk = np.clip(chunk, -1.0, 1.0)

                # Call KWS directly without VAD
                # But wait, WakeWordEngine.process calls both.
                # Let's just use engine.process()
                if engine.process(chunk):
                    print(f"Detected: {engine.last_keyword}", flush=True)
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        audio.stop()

if __name__ == "__main__":
    main()