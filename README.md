# Uncertainty-Aware Decision Making for Safe Navigation

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=for-the-badge&logo=ubuntu)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-FF7B00?style=for-the-badge)

## Overview
Standard autonomous navigation systems often rely solely on distance measurements to reach a waypoint. This project introduces a dynamic, uncertainty-aware navigation architecture. By calculating the population standard deviation of regional LiDAR (`/scan`) data, the robot evaluates the statistical uncertainty of its environment in real-time. 

As environmental clutter and measurement uncertainty increase, the robot autonomously throttles its linear velocity to prioritize safety, utilizing a hysteresis-based obstacle avoidance algorithm to escape local minima and reach dynamic coordinate goals.

<img width="1214" height="756" alt="WhatsApp Image 2026-10-07 at 11 17 11 PM" src="https://github.com/user-attachments/assets/d6c1cc8d-d6fd-4826-a0d7-2305ecfc9a05" />


## Core Features
* **Real-Time Uncertainty Estimation:** Divides LiDAR scans into discrete sectors (Front, Left, Right) and calculates standard deviation to classify environmental uncertainty as `LOW`, `MEDIUM`, or `HIGH`.
  
  <img width="1214" height="756" alt="WhatsApp Image 2026-10-07 at 11 18 42 PM" src="https://github.com/user-attachments/assets/e38fb2dd-95a7-40b3-9742-319510e90a3d" />

* **Adaptive Velocity Scaling:** Dynamically reduces linear speed by 25% to 50% based on the real-time uncertainty classification.
<img width="1215" height="743" alt="WhatsApp Image 2026-10-07 at 11 17 08 PM" src="https://github.com/user-attachments/assets/b50c6a5a-b715-423b-b820-61a4b32fefd5" />

  
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
```

## Execution Guide
This project runs across multiple terminals to separate simulation, perception, and control.

**1. Launch the Simulation Environment**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**2. Start the Perception Node (Uncertainty Estimator)**
```bash
ros2 run uncertainty_navigation uncertainty_estimator
```

**3. Start the Control Node (Goal Navigator)**
*Note: You can dynamically pass the target (x, y) coordinates using ROS 2 parameters.*
```bash
ros2 run uncertainty_navigation goal_navigator --ros-args -p goal_x:=-1.2 -p goal_y:=1.2
```

## Experimental Results
The system successfully proves that speed can be actively correlated with sensory uncertainty. Upon reaching the target coordinate, the `goal_navigator` node automatically halts and saves the telemetry to `navigation_results.csv`. 

To visualize the flight data, run the included plotting script:
<img width="1210" height="759" alt="WhatsApp Image 2026-10-07 at 11 17 15 PM (1)" src="https://github.com/user-attachments/assets/c4f5bd6f-e098-4cf3-a50c-94447ea512ca" />

```bash
python3 plot_results.py
```

## Author
**Barsharani Behera**  
B.Tech Computer Science Engineering
