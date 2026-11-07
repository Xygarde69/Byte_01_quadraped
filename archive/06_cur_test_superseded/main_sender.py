#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/main_sender.py
# STATUS    ·  WORKSPACE · latest experiments, not yet merged into robot/
# ────────────────────────────────────────────────────────────────────────
# Sends the initial standing pose to all four legs over the socket (the
# second step of the older start-up order: runtime -> main_sender ->
# gait/test script).
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 main_sender.py
# NOTE      ·  HOST is hard-coded to '0.0.0.0': works only when run on the
#              same machine as the runtime; use the runtime's IP
#              otherwise.
# ════════════════════════════════════════════════════════════════════════

import pickle
import socket
import time
from typing import Any, Dict, List


HOST = "0.0.0.0" #"127.0.0.1"
PORT = 50000

LEG_ORDER = ("fl", "bl", "fr", "br")
LegPayload = Dict[str, List[float]]
thigh = 0#30
knee = 0#30

def validate_leg_payload(payload: Any) -> LegPayload:
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict")

    normalized: LegPayload = {}
    
    for leg in LEG_ORDER:
        if leg not in payload:
            raise ValueError(f"missing leg '{leg}'")

        values = payload[leg]
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"payload['{leg}'] must be a list or tuple")

        if len(values) != 3:
            raise ValueError(f"payload['{leg}'] must have exactly 3 values")

        normalized[leg] = [float(v) for v in values]

    return normalized


def send_leg_angles(payload: Any, host: str = HOST, port: int = PORT) -> None:
    normalized = validate_leg_payload(payload)
    data = pickle.dumps(normalized, protocol=pickle.HIGHEST_PROTOCOL)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        sock.sendall(data)


def main() -> None:
    payload: LegPayload = {
        # will be transformed as per motor_config.json
        "fl": [85.0, -114.0-thigh, 6.0+knee], # 1, 2, 3 55 (60)  (145 65) 
        "bl": [85.0, -114.0-thigh, 2.0+knee], # 4, 5, 6 50
        "fr": [85.0, -114.0-thigh, 4.0+knee], # 7, 8, 9 55  (50)
        "br": [85.0, -116.0-thigh, 6.0+knee], # 10, 11, 12 50 (45)  hip all 2 degree more from straight
    }  

    send_leg_angles(payload)
    print("Sent grouped leg payload:")
    print(payload)


if __name__ == "__main__":
    main()
