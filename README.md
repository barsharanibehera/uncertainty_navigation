# Uncertainty-Aware Decision Making for Safe Navigation

## Overview
Standard autonomous navigation systems often rely solely on distance measurements to reach a waypoint. This project introduces a dynamic, uncertainty-aware navigation architecture. By calculating the population standard deviation of regional LiDAR (`/scan`) data, the robot evaluates the statistical uncertainty of its environment in real-time.

As environmental clutter and measurement uncertainty increase, the robot autonomously throttles its linear velocity to prioritize safety, utilizing a hysteresis-based obstacle avoidance algorithm to escape local minima and reach dynamic coordinate goals.

## Core Features
- **Real-Time Uncertainty Estimation:** Divides LiDAR scans into discrete sectors (Front, Left, Right) and calculates standard deviation to classify environmental uncertainty as `LOW`, `MEDIUM`, or `HIGH`.
- **Adaptive Velocity Scaling:** Dynamically reduces linear speed by 25% to 50% based on the real-time uncertainty classification.
- **Hysteresis Obstacle Avoidance:** Utilizes dual-threshold logic (Trigger: 0.60m, Clear: 0.85m) to prevent oscillatory "stuck" states when navigating tight corridors.
- **Goal Proximity Override:** Intelligently ignores physical obstacles that are located *behind* the target coordinate.
- **Automated Telemetry Logging:** Records odometry, sensory distance, uncertainty values, and command velocities to a CSV file at 10Hz for post-run analysis.

## System Requirements
- **OS:** Ubuntu 24.04 LTS
- **Framework:** ROS 2 Jazzy Jalisco
- **Simulation:** Gazebo Sim
- **Robot Model:** TurtleBot3 (Burger)
- **Dependencies:** `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `matplotlib` (for data visualization)

## Installation & Build

```bash
# Clone the repository into your ROS 2 workspace
cd ~/uncertainty_navigation_ws/src
git clone [https://github.com/barsharanibehera/uncertainty_navigation.git](https://github.com/barsharanibehera/uncertainty_navigation.git)

# Build the package
cd ~/uncertainty_navigation_ws
colcon build --packages-select uncertainty_navigation

# Source the workspace
source install/setup.bash   
