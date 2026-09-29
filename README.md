# AMR-POLEBOT-WS

> **Autonomous Mobile Robot (AMR) Development Workspace and Mission Control Stack**
> **Robotics and Automation Laboratory — Politeknik Manufaktur Bandung (POLMAN Bandung)**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-3498DB?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=flat-square)](LICENSE)
[![Build](https://img.shields.io/badge/Build-Colcon_Symlink-brightgreen?style=flat-square)](https://colcon.readthedocs.io/)
[![Hardware](https://img.shields.io/badge/Hardware-SocketCAN_500kbps-orange?style=flat-square)](https://www.kernel.org/doc/Documentation/networking/can.txt)

---

## Table of Contents

- [Overview](#overview)
- [System Specifications](#system-specifications)
- [Key Features](#key-features)
  - [Multi-Mode Motion Control](#1-modular-multi-mode-motion-control)
  - [Multi-Method Path Planning](#2-multi-method-global-path-planning)
  - [Web Mission Control Dashboard](#3-web-mission-control-dashboard)
  - [Confined Space Behavior Tree](#4-confined-space-behavior-tree)
  - [Costmap Filter Zones](#5-costmap-filter-zones)
- [System Architecture](#system-architecture)
- [Package Structure](#package-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#1-prerequisites)
  - [Build the Workspace](#2-build-the-workspace)
  - [Hardware Setup (CAN Bus)](#3-hardware-setup-can-bus)
  - [Launch Mission Control](#4-launch-mission-control)
  - [Operating the Robot](#5-operating-the-robot)
  - [Wireless Multi-Device Access](#6-wireless-multi-device-access)
- [Configuration Reference](#configuration-reference)
- [Contributors](#contributors)
- [License](#license)

---

## Overview

**AMR-POLEBOT** is an industrial-class differential drive Autonomous Mobile Robot designed and developed at **Politeknik Manufaktur Bandung (POLMAN Bandung)**. The platform serves as both a production-oriented AMR and a research testbed for comparing motion control strategies and path planning algorithms in real hardware.

This workspace integrates the full ROS 2 software stack for the robot, including:

- **Probabilistic localization** using AMCL with scan-matching corrected odometry
- **Intelligent navigation** via the Nav2 stack with customizable controllers and planners
- **Multi-mode motion control** — switchable at runtime between Native DWB, PID Profiled Pure Pursuit, and Sliding Mode Control (SMC)
- **Multi-method path planning** — switchable between NavFn A\*, BFS Grid Planner, and A\* Euclidean Optimal
- **A web-based 3D Mission Control Dashboard** built with Three.js, featuring real-time map rendering, costmap visualization, LiDAR point clouds, virtual zone editing, and sequential waypoint route planning
- **A custom Behavior Tree** designed for confined indoor spaces that limits recovery to a single backup attempt before terminating the goal
- **CANopen motor driver** for TongYi BLDC motors with precision odometry at 50 Hz

---

## System Specifications

| Parameter | Value |
|-----------|-------|
| Operating System | Ubuntu 24.04 LTS (Noble Numbat) |
| Robot Framework | ROS 2 Jazzy Jalisco |
| Drive System | 2x TongYi BLDC Direct Drive Motors + 4x Passive Caster Wheels (Differential Drive) |
| Motor Protocol | CANopen CiA 402 via SocketCAN (`can0` @ 500 kbps, node IDs 10 and 11) |
| Gear Ratio | 31.77:1 (measured) |
| Wheel Base | $W = 0.5473\text{ m}$ (calibrated from 3 m straight-line run) |
| Wheel Radius | $R = 0.079\text{ m}$ (158 mm tyre rolling radius) |
| Primary Sensor | Autonics LSC Series 2D LiDAR (Ethernet UDP at 192.168.0.1, 25 m range, 270-degree FoV) |
| Localization | AMCL (Adaptive Monte Carlo Localization) + SLAM Toolbox for mapping |
| Max Linear Velocity | $v_{\max} = 0.14\text{ m/s}$ |
| Max Angular Velocity | $\omega_{\max} = 0.18\text{ rad/s}$ ($\approx 10.3\text{ deg/s}$) |
| Linear Acceleration | $a_{\max} = 0.35\text{ m/s}^2$ |
| Control Frequency | Motor driver at 50 Hz, Nav2 controller at 20 Hz |
| Mission Dashboard | Web-based 3D GUI on port `5050` |
| Mobile Teleop | Offline joystick web app on port `8000` |

---

## Key Features

### 1. Modular Multi-Mode Motion Control

The robot supports three motion control modes that can be switched instantly from the web dashboard without restarting the navigation stack. A fourth "Standard OK" fallback button is provided as an emergency drawback mechanism.

**Native DWB (Default):**
The standard Nav2 DWB Local Planner with tuned critic weights for `RotateToGoal`, `PathAlign`, `GoalAlign`, `PathDist`, and `GoalDist`. This is the production-proven baseline controller that has been field-tested and validated. It uses a minimum angular speed threshold (`min_speed_theta: 0.08`) to prevent motor stalling during in-place rotation, and a `RotateToGoal.slowing_factor` of 1.5 for smooth deceleration near the goal heading.

**PID Profiled Pure Pursuit:**
A research controller based on S-Curve jerk-limited motion profiling ($j_{\max} = 0.16\text{ m/s}^3$) with gain-scheduled PID ($K_p = 3.20$ during cruise, $K_p = 3.00$ during deceleration) and curvature feedforward compensation. This controller is designed for smoother trajectory tracking with predictable acceleration profiles.

**Sliding Mode Controller (SMC):**
A nonlinear robust controller based on the approach described by Alipour et al. (2019). It uses polar-coordinate error sliding surfaces $S_1(\rho)$ and $S_2(\varphi)$ with hyperbolic tangent boundary layer switching to reduce chattering. Key parameters: $\lambda_1 = 0.5$, $\lambda_2 = 1.5$, $K_1 = 2.0$, $K_2 = 10.0$.

**Standard OK (Emergency Fallback):**
A one-click button that immediately terminates any active research controller process and reverts control to the stable Native DWB baseline. This serves as a safety drawback mechanism during experiments.

### 2. Multi-Method Global Path Planning

Three global path planning algorithms are available and can be switched from the dashboard:

- **Nav2 NavFn A\* (Default):** The standard Nav2 grid-based A\* planner operating on the 2D layered costmap. Reliable and well-tested for general indoor navigation.
- **BFS Grid Planner:** A Breadth-First Search wavefront planner with 8-connectivity on the occupancy grid. Deterministic and guaranteed to find the shortest grid path, though without cost optimization.
- **A\* Euclidean Optimal:** A weighted A\* planner with an obstacle distance-transform clearance penalty field. Produces paths that are not only short but also maintain clearance from walls and obstacles.

### 3. Web Mission Control Dashboard

The primary operator interface is a web-based 3D dashboard served on port `5050`. It is built with Three.js for 3D rendering and communicates with the ROS 2 stack via ROSBridge WebSocket (port `9090`) and a Python Flask REST API backend.

**3D Viewport:**
- Real-time rendering of the robot's 3D STL model on the occupancy grid map
- Global and local costmap overlay visualization
- LiDAR scan point cloud display
- Interactive camera controls (orbit, pan, zoom)

**Navigation Controls:**
- Motor initialization (CAN bus bring-up) via a single button click
- Map selection and Nav2 stack launch
- 2D Pose Estimate for initial localization
- Single-goal navigation with click-and-drag heading selection
- Mode selector dropdowns for motion controller and path planner

**Sequential Waypoint Route (RViz-style):**
- Place numbered waypoint targets sequentially on the 3D map (Point 1, 2, 3, ...)
- Circular billboard badges with route numbers that always face the camera
- Cyan neon route line connecting waypoints in order
- Waypoint list manager showing coordinates ($X$, $Y$, $\theta$) with per-point delete buttons and a clear-all button
- Route execution via the native `/navigate_through_poses` action
- Instant route cancellation support

**Virtual Zone Editor (Adobe-style):**
- Interactive polygon drawing tools for creating keepout zones (no-go areas) and speed restriction zones directly on the map
- Zones are applied as Nav2 costmap filter masks in real time

**Mobile Joystick Teleop (Port 8000):**
A separate lightweight web application providing a virtual joystick for manual teleoperation. Works offline on mobile devices connected to the same network.

### 4. Confined Space Behavior Tree

The custom Behavior Tree ([`polebot_obstacle_stop_and_backup.xml`](src/polebot_navigation/behavior_trees/polebot_obstacle_stop_and_backup.xml)) is specifically designed for compact indoor environments such as laboratory corridors and small rooms.

When the path is blocked by an obstacle, the robot executes the following sequence exactly once:

1. **Pause** for 0.5 seconds to settle chassis inertia
2. **Back up** 0.12 m at 0.05 m/s (gentle, safe clearance)
3. **Clear** the local costmap
4. **Terminate** the navigation goal immediately (`AlwaysFailure`)

The robot does **not** loop, retry, spin, or attempt to push through the obstacle. This conservative strategy prevents collisions with walls behind the robot in confined spaces where aggressive recovery maneuvers would be unsafe.

The Behavior Tree also supports runtime controller and planner selection through `ControllerSelector` and `PlannerSelector` nodes, enabling seamless integration with the dashboard's mode switching feature.

### 5. Costmap Filter Zones

The navigation stack supports two types of costmap filter overlays:

- **Keepout Filter:** Polygonal zones where the robot is prohibited from entering. Implemented as `nav2_costmap_2d::KeepoutFilter` on both global and local costmaps.
- **Speed Filter:** Polygonal zones where the robot's maximum velocity is reduced. Implemented as `nav2_costmap_2d::SpeedFilter`.

Both filter types can be drawn interactively using the web dashboard's zone editor and are stored as PGM/YAML mask files in the `maps/` directory.

---

## System Architecture

```
                    ┌──────────────────────────────────────────┐
                    │        Web Mission Control (5050)         │
                    │   Three.js 3D Viewport + REST API (Flask)│
                    └────────────────┬─────────────────────────┘
                                     │ HTTP + WebSocket
                    ┌────────────────┴─────────────────────────┐
                    │         ROSBridge WebSocket (9090)         │
                    └────────────────┬─────────────────────────┘
                                     │ ROS 2 Topics / Services / Actions
       ┌─────────────────────────────┼─────────────────────────────────┐
       │                             │                                 │
┌──────┴──────┐             ┌────────┴────────┐              ┌────────┴────────┐
│    AMCL     │             │   Nav2 Stack    │              │  SLAM Toolbox   │
│ Localization│             │ BT Navigator    │              │  (Mapping Mode) │
│             │             │ Controller Srv  │              │                 │
│ Particles:  │             │ Planner Server  │              └─────────────────┘
│ 500 – 2000  │             │ Recovery Server │
└──────┬──────┘             │ Costmap Filters │
       │                    └────────┬────────┘
       │                             │ /cmd_vel
       │                    ┌────────┴────────┐
       │                    │   TongYi CAN    │
       │ /scan              │  Open Driver    │
┌──────┴──────┐             │ SocketCAN 500k  │
│ Autonics    │             │ 50 Hz Control   │
│ LSC LiDAR   │             │ Diff Drive Odom │
│ Ethernet UDP│             └────────┬────────┘
└─────────────┘                      │ CAN Bus (can0)
                            ┌────────┴────────┐
                            │  Left Motor     │  Node ID 11
                            │  Right Motor    │  Node ID 10
                            └─────────────────┘
```

---

## Package Structure

```
AMR-POLEBOT-WS/
├── src/
│   ├── polebot_bringup/            # Robot bring-up launch files, CAN interface config, motor parameters
│   │   ├── config/
│   │   │   ├── tongyi_canopen_params.yaml   # Motor driver: gear ratio, wheel geometry, CAN node IDs
│   │   │   └── polebot_amr_mapper_params.yaml
│   │   └── launch/
│   │       ├── polebot.launch.py            # Full robot bring-up (motors + sensors + nav)
│   │       ├── polebot_motor.launch.py      # Motor-only bring-up
│   │       └── tongyi_lidar_slam.launch.py  # SLAM mapping session
│   │
│   ├── polebot_description/        # Robot model: URDF/Xacro, 3D STL meshes, wheel geometry
│   │   ├── meshes/                 # STL files: polebot_amr.stl, wheel.stl, caster.stl, lidar.stl, etc.
│   │   └── urdf/                   # Xacro files: polebot.urdf.xacro, polebot_base.xacro, etc.
│   │
│   ├── polebot_navigation/         # Nav2 configuration, behavior trees, costmap filters
│   │   ├── config/
│   │   │   └── nav2_params.yaml             # Full Nav2 parameter set (AMCL, controllers, planners, costmaps)
│   │   ├── behavior_trees/
│   │   │   └── polebot_obstacle_stop_and_backup.xml  # Custom confined-space BT
│   │   └── launch/
│   │       ├── navigation.launch.py
│   │       └── costmap_filters.launch.py
│   │
│   ├── polebot_slam/               # SLAM Toolbox 2D mapping configuration
│   │   ├── config/slam_toolbox_params.yaml
│   │   └── launch/slam.launch.py
│   │
│   ├── polebot_sensors/            # Sensor drivers and configuration
│   │   ├── config/lsc_lidar_params.yaml     # Autonics LSC LiDAR parameters
│   │   └── launch/
│   │       ├── lidar.launch.py
│   │       └── sensors.launch.py
│   │
│   ├── polebot_web_interface/      # Web Mission Control Dashboard (port 5050)
│   │   ├── polebot_web_interface/
│   │   │   └── web_backend.py               # Flask REST API backend + ROS 2 bridge
│   │   ├── www/
│   │   │   └── NavDashboard/
│   │   │       ├── index.html               # Dashboard HTML
│   │   │       ├── app.js                   # Main application logic (ROS topics, nav actions)
│   │   │       ├── workspace.js             # Three.js 3D viewport, map rendering, waypoints
│   │   │       └── styles.css               # UI stylesheet
│   │   └── launch/web_interface.launch.py
│   │
│   ├── polebot_web_teleop/         # Mobile joystick teleop web app (port 8000)
│   │
│   ├── polebot_research_control/   # Isolated research module folder
│   │   ├── polebot_research_control/
│   │   │   ├── pid/                         # PID Profiled Pure Pursuit controller nodes
│   │   │   │   ├── pitdt_profiled_pure_pursuit_controller_node.py
│   │   │   │   ├── path_profile_node.py
│   │   │   │   └── odom_to_posearray_node.py
│   │   │   ├── smc/                         # Sliding Mode Controller nodes
│   │   │   │   ├── sliding_mode_controller.py
│   │   │   │   ├── wheel_odom_publisher.py
│   │   │   │   └── odom_to_tf.py
│   │   │   └── path_planning/               # Alternative path planners
│   │   │       ├── bfs_planner.py
│   │   │       └── trajectory_mode_selector.py
│   │   ├── config/
│   │   │   ├── amcl_params.yaml
│   │   │   └── ros2_controllers.yaml
│   │   └── launch/research_control.launch.py
│   │
│   ├── polebot_control/            # Original control experiment nodes (legacy/reference)
│   │
│   ├── tongyi_canopen_driver/      # C++ SocketCAN CANopen DS402 motor driver
│   │   ├── launch/tongyi_bringup.launch.py
│   │   └── scripts/odom_echo.py
│   │
│   ├── lsc_ros2_driver/            # Autonics LSC LiDAR ROS 2 driver
│   │
│   └── polebot_simulation/         # Gazebo simulation world and test tracks
│       ├── config/ros_gz_bridge.yaml
│       └── launch/sim_gazebo.launch.py
│
├── maps/                           # Static map files
│   ├── Lab_Robotik.yaml / .pgm    # Primary lab map
│   ├── keepout_mask.yaml / .pgm   # Keepout zone overlay
│   ├── speed_mask.yaml / .pgm     # Speed restriction overlay
│   └── ...
│
├── docker/                         # Dockerfiles for isolated deployment
├── scripts/                        # Utility scripts (odometry calibration, CAN setup)
└── README.md
```

---

## Getting Started

### 1. Prerequisites

This workspace requires **Ubuntu 24.04 LTS** and **ROS 2 Jazzy Jalisco**.

Install the required system and ROS 2 packages:

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop-full \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rosbridge-server \
  can-utils iproute2
```

### 2. Build the Workspace

```bash
cd ~/Desktop/AMR-POLEBOT-WS
source /opt/ros/jazzy/setup.bash

# Build all packages with symlink install
colcon build --symlink-install

# Source the overlay workspace
source install/setup.bash
```

### 3. Hardware Setup (CAN Bus)

Before launching the motor driver, the SocketCAN interface must be initialized. This is normally handled automatically by the dashboard's "Start Motor" button, but can also be done manually:

```bash
# Bring up the CAN interface at 500 kbps
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000

# Verify the interface
candump can0
```

The motor driver communicates with two TongYi BLDC motors:
- **Left wheel:** CANopen node ID 11
- **Right wheel:** CANopen node ID 10

### 4. Launch Mission Control

A single launch command starts the entire stack: Flask backend, ROSBridge, static file server, and all ROS 2 nodes:

```bash
ros2 launch polebot_web_interface web_interface.launch.py
```

### 5. Operating the Robot

1. Open the dashboard in a browser at `http://localhost:5050`
2. Click **Start Motor** in the sidebar to initialize the CAN bus and enable the TongYi motor driver with calibrated kinematic parameters
3. Select the desired map (e.g., `Lab_Robotik.yaml`) and click **Load Map & Start Nav2**
4. Provide a 2D Pose Estimate if the robot's initial position is uncertain
5. Choose the navigation mode:
   - **Single Goal:** Click and drag on the map to set a target pose with heading
   - **Waypoint Route:** Switch to the Route tool to place sequential waypoints, then click **Start Route** to execute via `/navigate_through_poses`
6. Use the dropdown selectors to switch between motion controllers (Native DWB / PID / SMC) and path planners (NavFn A\* / BFS / A\* Euclidean) at any time during operation

### 6. Wireless Multi-Device Access

The dashboard binds to `0.0.0.0`, making it accessible from any device on the same network. To access the dashboard from a phone, tablet, or another laptop:

1. Connect the operator device to the same Wi-Fi hotspot or LAN as the robot
2. Find the robot's IP address: `hostname -I` (e.g., `10.86.182.19`)
3. Open the browser on the operator device and navigate to:
   - **Dashboard:** `http://<robot-ip>:5050`
   - **Mobile Joystick:** `http://<robot-ip>:8000`

The ROSBridge WebSocket connection (port `9090`) is resolved automatically using the browser's `window.location.hostname`.

---

## Configuration Reference

### Navigation Parameters

The main Nav2 configuration file is [`nav2_params.yaml`](src/polebot_navigation/config/nav2_params.yaml). Key tuned parameters:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `max_vel_x` | 0.14 m/s | Maximum forward speed |
| `max_vel_theta` | 0.18 rad/s | Maximum rotational speed |
| `min_speed_theta` | 0.08 rad/s | Minimum angular speed to prevent motor stall |
| `acc_lim_x` | 0.35 m/s^2 | Linear acceleration limit |
| `xy_goal_tolerance` | 0.15 m | Position goal tolerance |
| `yaw_goal_tolerance` | 0.15 rad | Heading goal tolerance |
| `RotateToGoal.slowing_factor` | 1.5 | Smooth deceleration near target heading |
| `inflation_radius` | 0.25 m | Costmap inflation around obstacles |

### Motor Driver Parameters

The motor configuration is in [`tongyi_canopen_params.yaml`](src/polebot_bringup/config/tongyi_canopen_params.yaml):

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `gear_ratio` | 31.77 | Measured gearbox ratio |
| `wheel_radius` | 0.079 m | Tyre rolling radius |
| `wheel_base` | 0.5473 m | Calibrated wheel separation |
| `max_motor_rpm` | 1000.0 | Motor speed limit |
| `control_hz` | 50.0 | Control loop frequency |
| `command_timeout_s` | 0.5 | Safety timeout for zero-velocity fallback |

### AMCL Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `max_particles` | 2000 | Upper particle count limit |
| `min_particles` | 500 | Lower particle count limit |
| `update_min_a` | 0.08 rad | Minimum angular displacement to trigger filter update |
| `update_min_d` | 0.15 m | Minimum linear displacement to trigger filter update |
| `laser_model_type` | likelihood_field | LiDAR measurement model |

---

## Contributors

The AMR-POLEBOT project is a collaborative effort between faculty researchers and student engineers at the **Robotics and Automation Laboratory, Department of Mechatronics Engineering, Politeknik Manufaktur Bandung (POLMAN Bandung)**.

### Faculty Advisors and Principal Researchers

| Name | Role | Affiliation / Profile |
|------|------|-----------------------|
| **Ismail, M.T.** | Head of Robotics and Automation Laboratory; Project Director and Mechatronics Systems | Politeknik Manufaktur Bandung |
| **Andri Wiyono, M.T.** | Faculty Researcher — Control Systems; AMR Drive Architecture and Differential Kinematics | Politeknik Manufaktur Bandung |
| **Siti Rodiah, M.T.** | Faculty Researcher — Intelligent Systems and Navigation; Path Planning Algorithms and Localization | [@rdhst](https://github.com/rdhst) |
| **Nur Jamiludin Ramadhan, M.T.** | Faculty Researcher — Robotics and Automation; Sensor Integration, Firmware, and Control Systems | [@nj-ramadhan](https://github.com/nj-ramadhan) |
| **Wahyu Caesarendra, Ph.D.** | Senior Researcher and Scientific Advisor; Autonomous Navigation and Intelligent Systems | [@WhyAC](https://github.com/WhyAC) |
| **Pipit Anggraeni, M.T.** | Faculty Researcher — Mechatronics; Instrumentation and Robot Dynamics Testing | Politeknik Manufaktur Bandung |
| **Noval, M.T.** | Faculty Researcher — Embedded Systems; CAN Bus Hardware Communication and Power Management | Politeknik Manufaktur Bandung |
| **Adhitya, M.T.** | Faculty Researcher — Robotics Instrumentation; Sensor Perception, LiDAR Safety, and Calibration | Politeknik Manufaktur Bandung |

### Student Engineering Developers

| Contributor | Role | Technical Focus |
|-------------|------|-----------------|
| **MiraeNK** | Lead Developer | System interfacing, web Mission Control Dashboard, Nav2 navigation tuning, odometry calibration |
| **Iridnes** | Developer | Motor control, differential drive kinematics, hardware integration |
| **RkZx** | Developer | 2D SLAM mapping, sensor setup, simulation validation |

---

## License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

<div align="center">
  <strong>Robotics and Automation Laboratory</strong><br>
  <strong>Politeknik Manufaktur Bandung (POLMAN Bandung)</strong><br>
  Jl. Kanayakan No. 21, Dago, Kecamatan Coblong, Kota Bandung, Jawa Barat 40135<br>
  <em>AMR-POLEBOT Autonomous Mobile Robot Project</em>
</div>
