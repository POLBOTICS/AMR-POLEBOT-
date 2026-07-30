# AMR-POLEBOT-WS

> **Official development workspace for the AMR-POLEBOT project**  
> Politeknik Manufaktur Bandung (POLMAN)

---

## 📋 Overview

AMR-POLEBOT is an Autonomous Mobile Robot (AMR) developed at Politeknik Manufaktur Bandung. This workspace manages the complete ROS 2 software stack for the robot, including navigation, sensor drivers, control systems, and simulation.

| Item | Detail |
|------|--------|
| **ROS Version** | ROS 2 Jazzy Jalisco |
| **OS** | Ubuntu 24.04 LTS |
| **Build System** | colcon |
| **Language** | Python (rclpy) + C++ (rclcpp) |

---

## 📦 Package Structure

```
AMR-POLEBOT-WS/
├── src/
│   ├── polebot_bringup/        # Launch files for full robot bring-up
│   ├── polebot_description/    # URDF/Xacro robot model & meshes
│   ├── polebot_navigation/     # Nav2 navigation stack config
│   ├── polebot_slam/           # SLAM Toolbox mapping config
│   ├── polebot_sensors/        # LiDAR, IMU, Camera drivers/config
│   ├── polebot_teleop/         # Keyboard & joystick teleoperation
│   ├── polebot_control/        # Custom control logic (C++)
│   ├── polebot_simulation/     # Gazebo simulation worlds
│   ├── polebot_dashboard/      # Web-based monitoring dashboard
│   └── polebot_interfaces/     # Custom ROS 2 msg/srv/action types
├── docker/                     # Dockerfiles for deployment
├── docs/                       # Documentation & diagrams
├── scripts/                    # Utility & setup scripts
└── README.md
```

---

## 🚀 Quick Start

### 1. Prerequisites

```bash
# Install ROS 2 Jazzy
sudo apt install ros-jazzy-desktop-full

# Install Nav2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# Install ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

### 2. Clone & Build

```bash
# Clone the workspace
git clone https://github.com/MiraeNK/AMR-POLEBOT-WS.git
cd AMR-POLEBOT-WS

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Launch

```bash
# Launch full robot (real hardware)
ros2 launch polebot_bringup polebot.launch.py

# Launch in simulation (Gazebo)
ros2 launch polebot_simulation polebot_sim.launch.py

# Launch SLAM mapping
ros2 launch polebot_slam slam.launch.py

# Launch navigation (with existing map)
ros2 launch polebot_navigation navigation.launch.py
```

---

## 📡 Topic Architecture

```
/polebot/
├── cmd_vel                  # Velocity commands (Twist)
├── odom                     # Odometry (Odometry)
├── scan                     # LiDAR scan (LaserScan)
├── imu/data                 # IMU data (Imu)
├── camera/image_raw         # Camera feed (Image)
├── battery_state            # Battery status (BatteryState)
└── diagnostics              # System diagnostics
```

---

## 🔗 Related Repositories

- **Motor Driver**: [AMR-POLEBOT Motor Driver](https://github.com/MiraeNK) — Hardware interface for motor controllers
- **Workspace**: [AMR-POLEBOT-WS](https://github.com/MiraeNK/AMR-POLEBOT-WS)

---

## 👥 Contributors

| Name | Role | Area |
|------|------|------|
| MiraeNK | Lead Developer | Interfacing & Mission Control |
| Iridnes | Developer | Motor Control & Limb Control |
| RkZx | Developer | SLAM Mapping & Navigation |

---

## 📄 License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE) for details.

---

<div align="center">
  <strong>Politeknik Manufaktur Bandung</strong><br>
  AMR-POLEBOT Project 🤖
</div>
