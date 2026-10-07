import csv
import os
import matplotlib.pyplot as plt

# File paths
csv_path = os.path.expanduser('~/navigation_results.csv')
save_path = os.path.expanduser('~/navigation_graph.png')

times = []
distances = []
linear_speeds = []
angular_speeds = []
uncertainties = []

print(f"Reading data from {csv_path}...")

with open(csv_path, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        times.append(float(row['Time']))
        distances.append(float(row['Distance_to_Goal']))
        linear_speeds.append(float(row['Linear_Speed']))
        # Added the angular speed (left/right) from the CSV
        angular_speeds.append(float(row['Angular_Speed']))
        uncertainties.append(float(row['Front_Std']))

# Create a figure with 4 subplots, making it taller to fit everything
plt.figure(figsize=(10, 11))

# 1. Distance Plot
plt.subplot(4, 1, 1)
plt.plot(times, distances, 'b-', linewidth=2)
plt.title('Distance to Goal Over Time', fontsize=12, fontweight='bold')
plt.ylabel('Distance (m)')
plt.grid(True)

# 2. Linear Speed Plot (Forward Movement)
plt.subplot(4, 1, 2)
plt.plot(times, linear_speeds, 'g-', linewidth=2)
plt.title('Forward Speed (Linear Velocity)', fontsize=12, fontweight='bold')
plt.ylabel('Speed (m/s)')
plt.grid(True)

# 3. Angular Speed Plot (Left/Right Movement)
plt.subplot(4, 1, 3)
plt.plot(times, angular_speeds, 'm-', linewidth=2)
plt.title('Left/Right Turning (Angular Velocity)', fontsize=12, fontweight='bold')
plt.ylabel('Speed (rad/s)')
plt.grid(True)
# Draw a zero line: Above 0 is turning Left, Below 0 is turning Right
plt.axhline(0, color='black', linewidth=1, linestyle='--')

# 4. Uncertainty Plot
plt.subplot(4, 1, 4)
plt.plot(times, uncertainties, 'r-', linewidth=2)
plt.title('LiDAR Measurement Uncertainty (Front Standard Deviation)', fontsize=12, fontweight='bold')
plt.xlabel('Time (seconds)')
plt.ylabel('Std Dev (m)')
plt.grid(True)

plt.tight_layout()
plt.savefig(save_path)
print(f"Success! Graph saved to {save_path}")
plt.show()
