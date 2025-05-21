# F1Tenth Autonomous Car - Senior Project

This repository contains the software and configuration files for our F1Tenth senior design project. The system enables an F1Tenth-scale autonomous car to map its environment, record waypoints, and drive autonomously using SLAM and waypoint following.

## Project Overview

Our F1Tenth vehicle uses:
- **SLAM Toolbox** for map creation and localization
- **EKF (Extended Kalman Filter)** for sensor fusion
- **Jetson GPIO** for PWM-based motor/servo control
- **Tachometers** for wheel-based odometry
- **Custom middleline generation** for optimal trajectory planning

---

## File Descriptions

### Source Code

- `cmd_vel_to_odom.py`  
  Publishes odometry based on `/cmd_vel` for visualization or backup localization.
  
- `drive_lap_launch.py`  
  Launch file to begin autonomous driving using waypoints and current pose.

- `jetson_gpio_control.py`  
  Publishes PWM signals to motor and servo via GPIO on Jetson Nano.

- `map_lap_launch.py`  
  Launch file for manual mapping using SLAM Toolbox and waypoint recording.

- `middleline.py`  
  Generates a smooth, central "middleline" path from a binary map using medial axis algorithms.

- `waypointfollower.py`  
  Main navigation node. Follows a pre-generated CSV of waypoints using pure pursuit logic.

- `pose_publisher.cpp`  
  Publishes estimated pose based on sensor or odometry fusion (used for EKF input) courtesy of Dr. Jonathan West.

- `tachometer_publisher.cpp`  
  Reads wheel encoder pulses and publishes odometry for localization and speed feedback.

---

### Configuration Files

- `ekf.yaml`  
  EKF parameters for fusing tachometer, IMU, and other odometry sources.

- `slam_localization.yaml`  
  SLAM Toolbox configuration in localization mode (used for `drive_lap` stage).

- `mapper_params_online_async.yaml`  
  SLAM Toolbox configuration for online mapping during manual lap.

---

### Build & Project Setup

- `CMakeLists.txt`  
  Build script for ROS 2 C++ nodes (`pose_publisher`, `tachometer_publisher`).
  
---

## System Workflow

1. **Map Lap**:  
   Run `map_lap_launch.py` to manually drive the car using teleop while SLAM builds a map and waypoints are recorded.

2. **Middleline Generation**:  
   Use `middleline.py` to generate a smooth trajectory from the map image and save to CSV.

3. **Autonomous Drive**:  
   Launch `drive_lap_launch.py` with localization and waypoint follower active.

---

## Dependencies

- ROS 2 Humble
- SLAM Toolbox
- Jetson.GPIO (Jetson Nano)
- OpenCV & NumPy (for `middleline.py`)
- `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs` (standard ROS 2 packages)
