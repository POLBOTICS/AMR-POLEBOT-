#!/bin/bash
# Script to install all dependencies for AMR-POLEBOT-WS
# Usage: ./setup_workspace.sh

echo "========================================="
echo "🤖 AMR-POLEBOT-WS Setup Script"
echo "========================================="

echo "[1/3] Updating apt package lists..."
sudo apt update

echo "[2/3] Updating rosdep..."
rosdep update

echo "[3/3] Installing ROS 2 dependencies automatically..."
# This command reads all package.xml files in the src/ directory 
# and installs missing packages (like rosbridge_server, slam_toolbox, etc.)
rosdep install --from-paths src --ignore-src -r -y

echo "========================================="
echo "✅ Setup Complete!"
echo "Now you can build the workspace with:"
echo "colcon build --symlink-install"
echo "========================================="
