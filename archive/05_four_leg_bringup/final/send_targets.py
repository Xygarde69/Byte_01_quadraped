#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/final/send_targets.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Sender library: converts logical joint-frame angles to raw motor
# angles using gear_ratio and flipped from motor_config.json.
# ════════════════════════════════════════════════════════════════════════
"""
ak60_send_targets.py
Sender library for ak60_homing_control.py.

Responsibility: convert logical joint-frame angles → raw motor frame
by applying gear_ratio and flipped from motor_config.json,
then send via socket.

ak60_homing_control.py operates purely in raw motor frame
and expects angles to already be converted before arrival.
"""

import socket
import pickle
import json
import numpy as np


DEFAULT_HOST        = '127.0.0.1'
DEFAULT_PORT        = 50000
DEFAULT_CONFIG_FILE = 'motor_config.json'

# Motor ID order expected by ak60_homing_control.py
MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]


def load_motor_config(config_file: str = DEFAULT_CONFIG_FILE) -> dict:
    """Load motor_config.json and return parsed dict."""
    with open(config_file, 'r') as f:
        return json.load(f)


def logical_to_raw(logical_angles: list | np.ndarray,
                   motor_config: dict) -> np.ndarray:
    """
    Convert logical joint-frame angles → raw motor frame.

    For each motor:
        raw = logical × gear_ratio
        raw = -raw  if flipped

    Parameters
    ----------
    logical_angles : 12-element array, degrees, logical joint frame
                     order: [M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12]
    motor_config   : parsed motor_config.json dict

    Returns
    -------
    np.ndarray of 12 raw motor angles in degrees
    """
    logical_angles = np.asarray(logical_angles, dtype=float)
    cfg = motor_config['motors']
    raw = np.zeros(len(MOTOR_IDS), dtype=float)

    for i, mid in enumerate(MOTOR_IDS):
        gr   = float(cfg[str(mid)]['gear_ratio'])
        flip = bool(cfg[str(mid)]['flipped'])
        raw[i] = logical_angles[i] * gr
        if flip:
            raw[i] = -raw[i]

    return raw


def _send_raw(raw_angles: np.ndarray,
              host: str, port: int, timeout: float) -> bool:
    """Internal: send a pre-converted raw angle array via socket."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((host, port))
        s.sendall(pickle.dumps(raw_angles))
        s.close()
        return True
    except ConnectionRefusedError:
        print(f"[send_targets] ❌ Connection refused — is ak60_homing_control.py running?")
        return False
    except TimeoutError:
        print(f"[send_targets] ❌ Timed out connecting to {host}:{port}")
        return False
    except Exception as e:
        print(f"[send_targets] ❌ Error: {e}")
        return False


def send_targets(
    logical_angles: list | np.ndarray,
    motor_config: dict  = None,
    config_file: str    = DEFAULT_CONFIG_FILE,
    host: str           = DEFAULT_HOST,
    port: int           = DEFAULT_PORT,
    timeout: float      = 5.0,
) -> bool:
    """
    Convert 12 logical joint-frame angles to raw motor frame and send.

    Parameters
    ----------
    logical_angles : [M1..M12] degrees in logical joint frame
    motor_config   : pre-loaded config dict (optional, loads from file if None)
    config_file    : path to motor_config.json (used if motor_config is None)
    host / port    : socket destination
    timeout        : connection timeout seconds

    Returns
    -------
    True on success, False on failure
    """
    if len(logical_angles) != 12:
        print(f"[send_targets] ❌ Expected 12 angles, got {len(logical_angles)}")
        return False

    if motor_config is None:
        motor_config = load_motor_config(config_file)

    raw = logical_to_raw(logical_angles, motor_config)

    print(f"[send_targets] Logical: {np.round(logical_angles, 2).tolist()}°")
    print(f"[send_targets] Raw:     {np.round(raw, 2).tolist()}°")

    ok = _send_raw(raw, host, port, timeout)
    if ok:
        print(f"[send_targets] ✅ Sent to {host}:{port}")
    return ok


def send_targets_per_leg(
    front_left:  list | np.ndarray,   # [M1,  M2,  M3 ] logical degrees
    back_left:   list | np.ndarray,   # [M4,  M5,  M6 ] logical degrees
    front_right: list | np.ndarray,   # [M7,  M8,  M9 ] logical degrees
    back_right:  list | np.ndarray,   # [M10, M11, M12] logical degrees
    motor_config: dict = None,
    config_file: str   = DEFAULT_CONFIG_FILE,
    host: str          = DEFAULT_HOST,
    port: int          = DEFAULT_PORT,
    timeout: float     = 5.0,
) -> bool:
    """
    Convenience wrapper — pass 3 logical angles per leg instead of flat 12.
    Leg order matches motor assignment:
        front_left  → M1,  M2,  M3
        back_left   → M4,  M5,  M6
        front_right → M7,  M8,  M9
        back_right  → M10, M11, M12
    """
    logical = np.concatenate([front_left, back_left, front_right, back_right],
                              dtype=float)
    return send_targets(logical,
                        motor_config=motor_config,
                        config_file=config_file,
                        host=host, port=port, timeout=timeout)


# ── Quick test when run directly ─────────────────────────────
if __name__ == "__main__":
    cfg = load_motor_config()

    # Example: IK solver output in logical joint frame
    send_targets_per_leg(
        front_left  = [60.0, -140.0, 35.0],   # M1,  M2,  M3  logical - will be flipped and scaled by gear ratio in send_targets()
        back_left   = [60.0, -140.0, 35.0],   # M4,  M5,  M6  logical - will be flipped and scaled by gear ratio in send_targets()
        front_right = [60.0, -140.0, 35.0],   # M7,  M8,  M9  logical - will be flipped and scaled by gear ratio in send_targets()
        back_right  = [60.0, -140.0, 35.0],   # M10, M11, M12 logical - will be flipped and scaled by gear ratio in send_targets()
        motor_config = cfg,
    )