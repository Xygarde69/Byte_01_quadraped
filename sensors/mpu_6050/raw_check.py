# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  sensors/mpu_6050/raw_check.py
# STATUS    ·  SENSOR BENCH TOOL · stand-alone, no robot needed
# ────────────────────────────────────────────────────────────────────────
# Smallest possible MPU-6050 test: wake the chip over I2C bus 1 (address
# 0x68) and print raw readings.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 raw_check.py
# ════════════════════════════════════════════════════════════════════════
import smbus2
import time
import math

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

# ===== CALIBRATION VALUES =====
ACCEL_BIAS = [-0.0823, 0.0097, -0.0068]
GYRO_BIAS  = [-5.0491, 1.4216, 0.4049]
# ===========================================

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def get_accel():
    ax = read_word(ACCEL_XOUT_H) / 16384.0 - ACCEL_BIAS[0]
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0 - ACCEL_BIAS[1]
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0 - ACCEL_BIAS[2]
    return ax, ay, az

def get_gyro():
    gx = read_word(GYRO_XOUT_H) / 131.0 - GYRO_BIAS[0]
    gy = read_word(GYRO_XOUT_H + 2) / 131.0 - GYRO_BIAS[1]
    gz = read_word(GYRO_XOUT_H + 4) / 131.0 - GYRO_BIAS[2]
    return gx, gy, gz

while True:
    ax, ay, az = get_accel()
    gx, gy, gz = get_gyro()

    print(
        f"Accel[g]: X={ax:+.3f} Y={ay:+.3f} Z={az:+.3f} | "
        f"Gyro[°/s]: X={gx:+.3f} Y={gy:+.3f} Z={gz:+.3f}"
    )
    time.sleep(0.1)