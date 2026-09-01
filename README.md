# 🤖 AMR-POLEBOT-WS

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
│   ├── polebot_web_teleop/     # 100% Offline Mobile Web Joystick UI
│   ├── polebot_simulation/     # Gazebo simulation worlds
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
git clone https://github.com/POLBOTICS/AMR-POLEBOT-.git
cd AMR-POLEBOT-WS

# Install dependencies (this will auto-install rosbridge_server + all others)
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

> **Note**: The `rosdep install` command automatically installs all ROS dependencies declared in `package.xml` files, including `rosbridge_server`. No need for separate installation!
>
> If you prefer manual installation: `sudo apt install ros-jazzy-rosbridge-server`

---

## ⚡ Motor Control & Web Teleop Guide

### **Option A: Manual Motor Control Only** (Testing/Debugging)

Use this for testing motor communication before running full robot stack.

```bash
# Terminal 1: Launch motor driver with CANopen interface
ros2 launch polebot_bringup polebot_motor.launch.py can_interface:=can0

# Optional: Auto-enable drives on startup (USE WITH CAUTION!)
# ros2 launch polebot_bringup polebot_motor.launch.py can_interface:=can0 auto_enable:=true
```

**Available motor commands:**

```bash
# Publish velocity commands to move the robot
ros2 topic pub /polebot/cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.2}"

# Stop the robot
ros2 topic pub /polebot/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"
```

---

### **Option B: Full Robot Bringup** (Production)

Launches motor driver + sensors + RViz visualization.

```bash
# Terminal 1: Full robot bringup
ros2 launch polebot_bringup polebot.launch.py can_interface:=can0

# Optional: Disable navigation and SLAM
# ros2 launch polebot_bringup polebot.launch.py can_interface:=can0 use_nav:=false use_slam:=false
```

---

### **Option C: Web Teleop Control via Local Hotspot** ⭐ (Recommended)

This is the **complete workflow** for controlling the robot from a client device via web UI over PC hotspot.

#### **Step 1: PC Setup (Robot PC)**

Make sure PC is running Ubuntu 24.04 LTS with ROS 2 Jazzy. Source the workspace first:

```bash
cd ~/AMR-POLEBOT-WS
source install/setup.bash
```

#### **Step 2: Verify CAN Interface**

```bash
# Check if CAN interface is available
ip link show can0

# If can0 doesn't exist, bring it up (adjust can0 to your interface)
sudo ip link set can0 up type can bitrate 500000
```

#### **Step 3: Launch Web Teleop Server**

In a terminal on the **Robot PC**, run:

```bash
# Terminal 1: Start web teleop (HTTP server + rosbridge_websocket)
ros2 launch polebot_web_teleop web_teleop.launch.py

# Output should show:
# [http.server]: Serving HTTP on 0.0.0.0:8000
# [rosbridge_websocket]: Started rosbridge WebSocket server on port 9090
```

#### **Step 4: Launch Motor Driver** (in another terminal)

```bash
# Terminal 2: Launch motor controller
ros2 launch polebot_bringup polebot_motor.launch.py can_interface:=can0
```

#### **Step 5: Setup PC Hotspot**

Share internet from Robot PC to create a WiFi hotspot that clients can connect to:

**Using GNOME Settings (Ubuntu GUI):**
1. Open **Settings** → **Network**
2. Click the **+** button to create a new connection
3. Select **Wi-Fi** → **Create...**
4. Configure:
   - **SSID**: `polebot-robot` (or your preferred name)
   - **Mode**: Hotspot
   - **Security**: WPA2 Personal
   - **Password**: `polebot2026` (or your preferred password)
5. Click **Create** and activate the hotspot

**Using Command Line:**

```bash
# Create hotspot
nmcli device wifi hotspot ifname wlan0 ssid "polebot-robot" password "polebot2026"

# View hotspot status
nmcli device show wlan0 | grep CONNECTION

# Stop hotspot
nmcli device disconnect wlan0
```

#### **Step 6: Find Robot PC IP Address**

Get the IP address of the robot PC on the hotspot network:

```bash
# Show all network interfaces
ip addr show

# Or find your wlan0 IP specifically
hostname -I
```

Example output: `192.168.100.XXX` (write this down)

---

### **Client Setup (Control Device)**

#### **Step 1: Connect to Robot Hotspot**

On your **client device** (laptop, tablet, phone):
1. Open WiFi settings
2. Select the hotspot `polebot-robot`
3. Enter password: `polebot2026`

#### **Step 2: Access Web Teleop UI**

Open your browser and navigate to:

```
http://192.168.100.XXX:8000
```

Replace `192.168.100.XXX` with the Robot PC IP address found in the previous step.

**Expected Output:**
- A dark web UI appears with a circular **joystick zone**
- Status indicator showing "Connected" (green) or "Disconnected" (red)
- Speed control buttons (Slow / Normal / Fast)

#### **Step 3: Control the Robot**

- **Drag the joystick** inside the circular zone to move the robot
- **Drag left/right** for forward/backward movement
- **Drag up/down** for rotation (turn left/right)
- **Release joystick** to stop the robot
- Click **speed buttons** to adjust velocity multiplier

---

### **Troubleshooting Web Teleop Connection**

#### Issue: "Cannot connect to server"

```bash
# Check if HTTP server is running
curl http://192.168.100.XXX:8000

# Check if rosbridge is running
netstat -tuln | grep 9090
# Should show: LISTEN on port 9090
```

#### Issue: Cannot reach Robot PC IP

```bash
# Verify you're connected to the hotspot
nmcli device show wlan0 | grep IP4.ADDRESS

# Test connectivity from client
ping 192.168.100.XXX
```

#### Issue: Joystick not responding

1. Check browser console (F12 → Console tab) for errors
2. Verify rosbridge_websocket is running: `ros2 node list`
3. Verify `/cmd_vel` topic exists: `ros2 topic list | grep cmd_vel`
4. Manually publish cmd_vel to test:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" --rate 10
   ```

---

### 3. Full Robot Launch (Original)

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
- **Workspace**: [AMR-POLEBOT-WS](https://github.com/POLBOTICS/AMR-POLEBOT-.git)

---

## 👥 Contributors

| Name    | Role           | Area                                        | 
|---------|----------------|---------------------------------------------|
| MiraeNK | Lead Developer | Interfacing, Mission Control, & Navigation  |
| Iridnes | Developer      | Motor Control & Diff Drive                  |
| RkZx    | Developer      | SLAM Mapping & Navigation                   | 

---

## 📄 License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE) for details.

---

<div align="center">
  <strong>Politeknik Manufaktur Bandung</strong><br>
  AMR-POLEBOT Project 
</div>
