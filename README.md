#  Buggerbot — Gazebo Sim (gz-sim) Based Simulation

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-blue?style=for-the-badge&logo=ros" />
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" />
  <img src="https://img.shields.io/badge/Open%20Source-❤️-brightgreen?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" />
</p>

<p align="center">
  <!-- ADD: A screenshot or GIF of Buggerbot running in Gazebo -->
  <img src="/home/sharva45/Pictures/Screenshots/buggerbot screenshot.png" alt="Buggerbot in Gazebo Simulation" width="700"/>
</p>

> A complete **ROS 2 Humble + Gazebo Harmonic** simulation of Buggerbot — a differential drive wheeled robot — with a full URDF/SDF model, LiDAR/Camera/IMU sensor suite, keyboard teleoperation, and Nav2 autonomous navigation.

---

##  Table of Contents

- [Features](#-features)
- [Demo](#-demo)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Package Structure](#-package-structure)
- [Usage](#-usage)
- [Sensor Topics](#-sensor-topics)
- [Contributing](#-contributing)
- [License](#-license)

---

## ✨ Features

-  **Differential Drive** — Two-wheeled robot with full wheel odometry
-  **URDF / SDF Model** — Parameterized robot description with meshes and inertial properties
-  **Sensor Suite** — LiDAR, RGB Camera, and IMU publishing to standard ROS 2 topics
-  **Teleoperation** — Keyboard-based manual control
-  **Nav2 Navigation** — Autonomous waypoint navigation with costmaps and path planning
-  **Gazebo Harmonic** — Modern gz-sim with high-fidelity physics and rendering

---

##  Demo

<!-- ADD: Screenshots of robot in Gazebo world and RViz2 -->
| Gazebo Simulation | RViz2 Navigation |
|:-----------------:|:----------------:|
| ![sim](/home/sharva45/Pictures/Screenshots/buggerbotsim.png) | ![rviz](/home/sharva45/Pictures/Screenshots/buggerbotrviz.png) |

<!-- ADD: Optional video embed or link -->
>  [Watch full demo video](<!-- ADD: YouTube or Google Drive link here -->)

---

##  Prerequisites

| Dependency | Version |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Gazebo Sim | Harmonic |
| `ros_gz` bridge | Humble-compatible |
| Nav2 | Latest for Humble |

```bash
# Install Gazebo Harmonic
sudo apt-get install gz-harmonic

# Install ROS dependencies
sudo apt install ros-humble-ros-gz \
                 ros-humble-nav2-bringup \
                 ros-humble-navigation2 \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher
```

---

##  Installation

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone the repo
git clone https://github.com/Sharva45/Buggerbot-gz_sim-based-simulation.git

# 3. Install dependencies
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install

# 5. Source
source install/setup.bash
```

---


> ⚠️ Update the structure above to match your actual directory layout.

---

##  Usage

Source your workspace before running anything:
```bash
source ~/ros2_ws/install/setup.bash
```

### 1. Launch Simulation

```bash
# ADD: your actual launch command here
ros2 launch bugger gz_simulator_launch.py
```

<!-- ADD: Screenshot of Gazebo opening with robot spawned -->

---

### 2. Teleoperation

```bash
# ADD: your teleop command here
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=<YOUR_CMD_VEL_TOPIC>
```

<!-- ADD: GIF of robot being driven with keyboard -->

---

### 3. Autonomous Navigation (Nav2)

```bash
#Check the 
# ADD: your navigation launch command here
ros2 launch bugger navigation.launch.py
```

<!-- ADD: Screenshot of Nav2 running in RViz2 with path visible -->

---

## 📡 Sensor Topics

| Sensor | Topic | Message Type |
|--------|-------|--------------|
| LiDAR | `<!-- ADD topic -->` | `sensor_msgs/LaserScan` |
| Camera | `<!-- ADD topic -->` | `sensor_msgs/Image` |
| IMU | `<!-- ADD topic -->` | `sensor_msgs/Imu` |
| Odometry | `<!-- ADD topic -->` | `nav_msgs/Odometry` |
| Cmd Vel | `<!-- ADD topic -->` | `geometry_msgs/Twist` |

> Run `ros2 topic list` after launching to confirm your actual topic names.

---

##  Contributing

**This is an open source project — you are free to use, modify, and build upon it!**

All contributions are welcome, whether it's bug fixes, new features, better docs, or new worlds and maps.

1. Fork the repository
2. Create your branch: `git checkout -b feature/your-feature`
3. Commit your changes: `git commit -m 'Add your feature'`
4. Push to your branch: `git push origin feature/your-feature`
5. Open a Pull Request

Found a bug or have an idea? [Open an issue](https://github.com/Sharva45/Buggerbot-gz_sim-based-simulation/issues) — feedback of any kind is appreciated.

---

##  License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

You are free to use this project for personal, academic, or commercial purposes.

---

<p align="center">Built with ❤️ using ROS 2 Humble & Gazebo Harmonic &nbsp;|&nbsp; Open Source & Free to Use</p>