Overview

The robot uses a four‑wheel differential‑drive chassis equipped with:

A 2‑D LiDAR (e.g., RPLIDAR A2) providing range scans.
An IMU (e.g., MPU‑6050) publishing raw linear acceleration and angular velocity.
The RF2O laser odometry node (C++/ROS 2) supplies high‑frequency LiDAR odometry, which is then fused with the IMU data in the Python helper scripts (ddsm.py, imu.py, yd2_fixed.py) to correct for drift and generate a smooth pose estimate.
The fused pose is fed to slam_toolbox (ROS 2) to produce a 2‑D occupancy‑grid map in real time.
The rf2o_laser_odometry package is not included in the current upload; you can add it later as a sub‑module or copy the source into src/rf2o_laser_odometry.

Prerequisites
ROS 2 jazzy
colcon build tool
python3
numpy, scipy, pyyaml
rplidar_ros2 (or your LiDAR driver)
imu_pose_estimation (or any ROS2 IMU driver)
slam_toolbox

Configuration
All tunable parameters live under src/my_robot_slam/config/:
ekf.yaml - EKF settings for fusing LiDAR odometry and IMU data (process noise, sensor noise, initial covariances).
rf2o_params.yaml - RF2O laser odometry parameters (scan topic, max range, resolution, etc.).
slam_toolbox_params.yaml - slam_toolbox configuration (map resolution, publish rates, loop‑closure settings)
Edit these YAML files to match your hardware (e.g., LiDAR frame IDs, IMU noise values) before the first run.
