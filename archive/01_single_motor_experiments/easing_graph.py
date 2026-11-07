# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/easing_graph.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Plots the easing curves (ease_in_cubic and a custom exponent) used to
# shape motor motion.
# ════════════════════════════════════════════════════════════════════════
import math
import numpy as np
import matplotlib.pyplot as plt

def ease_in_cubic(x: float) -> float:
    return 1 - math.pow(1 - x, 3)

def ease_in_custom(x: float) -> float:
    return 1 - math.pow(1 - x, 2.5)

# Generate x values from 0 to 1
x = np.linspace(0, 1, 100)

y_cubic = [ease_in_cubic(val) for val in x]
y_custom = [ease_in_custom(val) for val in x]

# Plot
plt.figure()
plt.plot(x, y_cubic, label="Ease In Cubic (power=3)")
plt.plot(x, y_custom, label="Ease In Custom")

plt.xlabel("x")
plt.ylabel("output")
plt.title("Ease In Function Comparison")
plt.legend()
plt.grid(True)
plt.show()
