#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/imu_correction.py
# STATUS    ·  WORKSPACE · latest experiments, not yet merged into robot/  (NEW, untracked)
# ────────────────────────────────────────────────────────────────────────
# Wraps MPULeveler in a background thread that polls the MPU-6050 and
# exposes per-leg Y corrections via a thread-safe get(); also bundles
# two start-up health gates: validate_mpu() and validate_socket(). Used
# by kriss_trot_controller.py --imu.
# ════════════════════════════════════════════════════════════════════════
"""
imu_correction.py
─────────────────────────────────────────────────────────────────────────────
Continuous MPU-6050 chassis leveling for BYTE-01.

Wraps MPULeveler in a background thread that polls the sensor and exposes
per-leg Y corrections via a thread-safe getter.  Also bundles the two
startup health gates (MPU read validation + socket reachability).

Usage (in a gait script with --imu):
    from imu_correction import IMUCorrector

    corrector = IMUCorrector(leveler, ALL_LEGS, STANDING_XYZ, _kill_flag)

    # pre-flight (call before starting gait):
    if not corrector.validate_mpu():
        pygame.quit(); sys.exit(1)
    if not corrector.validate_socket(HOST, PORT):
        pygame.quit(); sys.exit(1)

    corrector.start()                  # launch background thread

    # inside gait / tilt loop:
    corr = corrector.get()             # {"fl": dy, "fr": dy, ...}
"""

import socket
import threading
import time

# ── tunables ──────────────────────────────────────────────────────────────────
IMU_UPDATE_HZ    = 5.0    # poll rate for the correction thread
IMU_SAMPLES      = 10     # accel samples averaged per read (~50 ms)
IMU_CORR_MAX     = 6.0    # cm — safety clamp on per-leg Y correction
MPU_HEALTH_READS = 50     # consecutive successful reads required at startup


class IMUCorrector:
    """
    Background MPU poller + per-leg Y correction store.

    Args:
        leveler     : MPULeveler instance (already initialised, reference locked)
        all_legs    : list of leg names, e.g. ["fl", "fr", "bl", "br"]
        standing_xyz: [x, y, z] nominal standing foot position; [1] is the Y baseline
        kill_flag   : threading.Event — set to stop the background thread
    """

    def __init__(self, leveler, all_legs, standing_xyz, kill_flag):
        self._leveler     = leveler
        self._all_legs    = list(all_legs)
        self._standing_y  = standing_xyz[1]
        self._kill_flag   = kill_flag
        self._corr        = {leg: 0.0 for leg in all_legs}
        self._lock        = threading.Lock()
        self._thread      = None

    # ── pre-flight gates ──────────────────────────────────────────────────────

    def validate_mpu(self, n: int = MPU_HEALTH_READS) -> bool:
        """
        Read the MPU n times and confirm every read succeeds with plausible
        values.  Prints per-failure details; summary on completion.
        """
        print(f"[IMU] MPU health check — {n} consecutive reads required...")
        failures = 0
        for i in range(n):
            try:
                raw = self._leveler._read_raw_accel()
                if raw == (0, 0, 0):
                    failures += 1
                    print(f"[IMU] Read {i+1}: all-zero (sensor unresponsive)")
                elif any(abs(v) > 32767 for v in raw):
                    failures += 1
                    print(f"[IMU] Read {i+1}: out-of-range {raw}")
            except Exception as e:
                failures += 1
                print(f"[IMU] Read {i+1} FAILED: {e}")
            time.sleep(0.005)

        if failures == 0:
            print(f"[IMU] MPU OK — {n}/{n} reads clean.")
            return True
        print(f"[IMU] MPU FAILED — {failures}/{n} reads errored.")
        return False

    def validate_socket(self, host: str, port: int,
                        timeout: float = 1.0) -> bool:
        """
        TCP-connect to main_with_ik.py to confirm the motor pipeline is up.
        main_with_ik only opens this port AFTER verify_periodic_feedback_and_
        capture_boot_holds() and homing complete, so a successful connect
        transitively confirms motor feedback is healthy.
        """
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect((host, port))
            print(f"[IMU] Motor controller reachable at {host}:{port}.")
            return True
        except Exception as e:
            print(f"[IMU] Motor controller unreachable at {host}:{port}: {e}")
            return False

    # ── background thread ─────────────────────────────────────────────────────

    def _run(self):
        dt = 1.0 / IMU_UPDATE_HZ
        while not self._kill_flag.is_set():
            try:
                targets = self._leveler.compute_level_targets(
                    n_samples=IMU_SAMPLES, verbose=False
                )
                new_corr = {
                    leg: max(-IMU_CORR_MAX,
                             min(IMU_CORR_MAX, targets[leg][1] - self._standing_y))
                    for leg in self._all_legs
                }
                with self._lock:
                    self._corr = new_corr
            except Exception as e:
                print(f"\r[IMU] read error: {e}", flush=True)
            time.sleep(dt)

    def start(self):
        """Launch the correction thread (call after capture_reference())."""
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="IMUCorrector")
        self._thread.start()
        print(f"[IMU] Correction thread started at {IMU_UPDATE_HZ:.0f} Hz.")

    def join(self, timeout: float = 2.0):
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    # ── getter ────────────────────────────────────────────────────────────────

    def get(self) -> dict:
        """
        Return a thread-safe snapshot of per-leg Y corrections (cm).
        Positive value → leg extends more (pushes that corner of the body up).
        """
        with self._lock:
            return dict(self._corr)
