# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/check_audio.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# Prints the list of audio devices PortAudio can see (sanity check for
# the microphone).
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 check_audio.py
# ════════════════════════════════════════════════════════════════════════
import sounddevice as sd
import numpy as np

print("--- Starting Audio Check ---")
try:
    devices = sd.query_devices()
    if not devices:
        print("Error: PortAudio returned an empty list.")
    else:
        print(devices)
except Exception as e:
    print(f"Caught an error during query: {e}") 