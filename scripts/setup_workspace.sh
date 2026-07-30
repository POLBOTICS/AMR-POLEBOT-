#!/bin/bash
# =============================================================
#  POLEBOT Workspace Setup Script
#  AMR-POLEBOT | Politeknik Manufaktur Bandung
#  ROS 2 Jazzy | Ubuntu 24.04
# =============================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROS_DISTRO="jazzy"

echo "=============================================="
echo " POLEBOT Workspace Setup"
echo " Workspace: $WORKSPACE_DIR"
echo " ROS: $ROS_DISTRO"
echo "=============================================="

# Source ROS 2
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    echo "[OK] ROS 2 ${ROS_DISTRO} sourced"
else
    echo "[ERROR] ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
    echo "Please install ROS 2 Jazzy first."
    exit 1
fi

# Install dependencies
echo ""
echo "[1/3] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-imu-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool

echo "[OK] System dependencies installed"

# rosdep
echo ""
echo "[2/3] Running rosdep..."
cd "$WORKSPACE_DIR"

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi

rosdep update
rosdep install --from-paths src --ignore-src -r -y
echo "[OK] rosdep dependencies satisfied"

# Build
echo ""
echo "[3/3] Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo ""
echo "=============================================="
echo " Build complete!"
echo ""
echo " To activate workspace, run:"
echo "   source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo " Quick launch commands:"
echo "   # Full robot bringup:"
echo "   ros2 launch polebot_bringup polebot.launch.py"
echo ""
echo "   # SLAM mapping:"
echo "   ros2 launch polebot_bringup polebot.launch.py use_slam:=true use_nav:=false"
echo ""
echo "   # Simulation:"
echo "   ros2 launch polebot_simulation polebot_sim.launch.py"
echo ""
echo "   # Keyboard teleop:"
echo "   ros2 run polebot_teleop keyboard_teleop"
echo "=============================================="
