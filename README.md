# Uncertainty-Aware Decision Making for Safe Navigation

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=for-the-badge&logo=ubuntu)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-FF7B00?style=for-the-badge)

## Overview
Standard autonomous navigation systems often rely solely on distance measurements to reach a waypoint. This project introduces a dynamic, uncertainty-aware navigation architecture. By calculating the population standard deviation of regional LiDAR (`/scan`) data, the robot evaluates the statistical uncertainty of its environment in real-time. 

As environmental clutter and measurement uncertainty increase, the robot autonomously throttles its linear velocity to prioritize safety, utilizing a hysteresis-based obstacle avoidance algorithm to escape local minima and reach dynamic coordinate goals.

## Core Features
* **Real-Time Uncertainty Estimation:** Divides LiDAR scans into discrete sectors (Front, Left, Right) and calculates standard deviation to classify environmental uncertainty as `LOW`, `MEDIUM`, or `HIGH`.
* **Adaptive Velocity Scaling:** Dynamically reduces linear speed by 25% to 50% based on the real-time uncertainty classification.
* **Hysteresis Obstacle Avoidance:** Utilizes dual-threshold logic (Trigger: 0.60m, Clear: 0.85m) to prevent oscillatory "stuck" states when navigating tight corridors.
* **Goal Proximity Override:** Intelligently ignores physical obstacles that are located *behind* the target coordinate.
* **Automated Telemetry Logging:** Records odometry, sensory distance, uncertainty values, and command velocities to a CSV file at 10Hz for post-run analysis.

## System Requirements
* **OS:** Ubuntu 24.04 LTS
* **Framework:** ROS 2 Jazzy Jalisco
* **Simulation:** Gazebo Sim
* **Robot Model:** TurtleBot3 (Burger)
* **Dependencies:** `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `matplotlib` (for data visualization)

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

Execution Guide
This project runs across multiple terminals to separate simulation, perception, and control.

1. Launch the Simulation Environment

Bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. Start the Perception Node (Uncertainty Estimator)

Bash
ros2 run uncertainty_navigation uncertainty_estimator
3. Start the Control Node (Goal Navigator)
Note: You can dynamically pass the target (x, y) coordinates using ROS 2 parameters.

Bash
ros2 run uncertainty_navigation goal_navigator --ros-args -p goal_x:=-1.2 -p goal_y:=1.2
Experimental Results
The system successfully proves that speed can be actively correlated with sensory uncertainty. Upon reaching the target coordinate, the goal_navigator node automatically halts and saves the telemetry to navigation_results.csv.

To visualize the flight data, run the included plotting script:

Bash
python3 plot_results.py
(You can upload the navigation_graph.png file to your repository and display it here to show the 4-part graph proving the speed drop during high-uncertainty events).

Author
Barsharani Behera

Author
Barsharani Behera
B.Tech Computer Science Engineering
