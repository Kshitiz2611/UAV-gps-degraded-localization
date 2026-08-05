# GPS-Degraded Multi-Source UAV Localization


This repository contains the software for a **GPS-degraded multi-source UAV localization system** designed to operate in indoor and semi-indoor environments. The system fuses **LiDAR SLAM (LIO-SAM)**, **Visual Inertial Odometry (VINS-Fusion)**, and **GPS/IMU (PX4 EKF2)** using covariance-gated inverse-weight fusion with chi-squared innovation gating.

Validated in **Gazebo Harmonic simulation** with PX4 SITL, achieving automatic sensor weight transitions — LIO-SAM weight adapts from 0.19% to 25% (GPS-present mode) and up to 64% (GPS-degraded mode) based on real-time feature confidence.

---

## Core Features

* **Covariance-Gated Fusion:** Chi-squared innovation gating (95% confidence) rejects inconsistent sensor measurements before fusion — no hard switching, continuous weighted blending.
* **Proxy Covariance Estimation:** LIO-SAM and VINS-Fusion both publish zero covariance in their ROS2 ports. The system infers uncertainty from LiDAR feature count and visual tracked point count.
* **6-State Graceful Degradation:** Automatic state transitions between FULL → LIO_DEGRADED → VIO_DEGRADED → GPS_DEGRADED → LIO_VIO_FAILED → ALL_FAILED based on real-time sensor health.
* **Timestamp Synchronization:** Resolves 5-second visual frame rendering latency from Gazebo Harmonic's ogre2 pipeline by restamping camera frames with IMU timestamps, enabling correct VINS-Fusion IMU preintegration.
* **Loop Closure:** LIO-SAM uses Scan Context place recognition + GTSAM factor graph for globally consistent mapping — confirmed in simulation.

---

## Tech Stack

* **Core Logic:** Python 3.10+
* **Robotics Framework:** ROS2 Humble
* **Simulation:** Gazebo Harmonic 8.14 + PX4 SITL
* **LiDAR SLAM:** LIO-SAM (ros2 branch) + GTSAM 4.1
* **Visual Odometry:** VINS-Fusion stereo+IMU (zinuok ROS2 port) + Ceres 2.2.0
* **Flight Controller:** PX4 EKF2 (GPS+IMU)
* **Libraries:** NumPy, rclpy, px4_msgs

---


## Simulation World

The system is validated in a custom **walls world** (`walls.sdf`) — an enclosed brick-textured room providing:
- Clean planar features for LIO-SAM LOAM feature extraction
- Not reliable for ivisual corners for VINS-Fusion KLT tracking | In progress to add new distinct features like warehouse
- GPS-denied-like indoor geometry for testing degradation states

---

## Files

| File | Description |
|------|-------------|
| `fusion_node.py` | Fusing odom and gating the covariance |
| `lio_sam_params.yaml` | LIO-SAM config for VLP-16 16-beam simulation |
| `vins_config.yaml` | VINS-Fusion stereo+IMU config for Gazebo x500 |
| `walls.sdf` | Custom simulation world with brick-textured walls |

---


### Installation

1. **Clone the repository:**
```bash
git clone https://github.com/YOUR_USERNAME/uav-gps-degraded-localization.git
cd uav-gps-degraded-localization
```

2. **Build ROS2 packages:**
```bash
cd ~/ros2_ws
colcon build --packages-select lio_sam vins sensor_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Usage

**Terminal 1 — PX4 SITL + Gazebo:**
```bash
cd ~/PX4-Autopilot && PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_3d
```

**Terminal 2 — DDS Bridge:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — LIO-SAM:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lio_sam run.launch.py
```

**Terminal 4 — VINS-Fusion:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run vins vins_node ~/ros2_ws/src/VINS-Fusion-ROS2/config/gazebo_x500/vins_config.yaml
```

**Terminal 5 — Fusion Node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run sensor_fusion fusion_node --ros-args -p use_sim_time:=true
```

**Monitor fusion state:**
```bash
ros2 topic echo /fusion/state
ros2 topic echo /fusion/weights
```

---

## Fusion States

| State | Condition | Primary Source |
|-------|-----------|----------------|
| FULL | All three healthy | LIO-SAM + GPS + VIO |
| LIO_DEGRADED | LIO-SAM features insufficient | GPS + VIO |
| VIO_DEGRADED | VINS tracking lost | LIO-SAM + GPS |
| GPS_DEGRADED | GPS fix lost | LIO-SAM + VIO |
| LIO_VIO_FAILED | Both LIO and VIO down | GPS only |
| ALL_FAILED | All sources failed | IMU dead reckoning |
