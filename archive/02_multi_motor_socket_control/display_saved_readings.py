# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/display_saved_readings.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Reads motor_positions.txt and displays the recorded motor positions.
# ════════════════════════════════════════════════════════════════════════
import matplotlib.pyplot as plt
import numpy as np

# Read the data from the text file
filename = 'motor_positions.txt'  # Replace with your actual filename
with open(filename, 'r') as f:
    lines = f.readlines()

# Parse the data
data1 = []  # Series 1 values
data2 = []  # Series 2 values  
data3 = []  # Series 3 values
time_steps = []

for i, line in enumerate(lines):
    if line.strip():  # Skip empty lines
        parts = line.strip().split(',')
        values = []
        for part in parts:
            if ':' in part:
                val = float(part.split(':')[1])
                values.append(val)
        
        if len(values) >= 3:
            data1.append(values[0])
            data2.append(values[1])
            data3.append(values[2])
            time_steps.append(i+1)

# Convert to numpy arrays
x = np.array(time_steps)
y1 = np.array(data1)
y2 = np.array(data2)
y3 = np.array(data3)

# Create the plot
plt.figure(figsize=(12, 8))
plt.plot(x, y1, 'b-', label='Series 1', linewidth=2)
plt.plot(x, y2, 'r-', label='Series 2', linewidth=2)
plt.plot(x, y3, 'g-', label='Series 3', linewidth=2)

# Customize the plot
plt.xlabel('Time Step')
plt.ylabel('Value')
plt.title('Time Series Data Plot')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Show the plot
plt.show()
