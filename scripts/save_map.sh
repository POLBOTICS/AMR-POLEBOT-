#!/bin/bash
# =============================================================
#  POLEBOT Map Saver Script
#  Simpan hasil SLAM mapping ke folder maps/
#  Usage: ./save_map.sh [map_name]
# =============================================================

MAP_NAME="${1:-polman_lab_$(date +%Y%m%d_%H%M%S)}"
MAP_DIR="$(cd "$(dirname "$0")/.." && pwd)/maps"

mkdir -p "$MAP_DIR"

echo "=============================================="
echo " Saving map: $MAP_NAME"
echo " Location:   $MAP_DIR/$MAP_NAME"
echo "=============================================="

source /opt/ros/jazzy/setup.bash

ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p save_map_timeout:=5.0

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Map saved:"
    echo "  - $MAP_DIR/$MAP_NAME.pgm"
    echo "  - $MAP_DIR/$MAP_NAME.yaml"
    echo ""
    echo "To use for navigation:"
    echo "  ros2 launch polebot_navigation navigation.launch.py \\"
    echo "    map:=$MAP_DIR/$MAP_NAME.yaml"
else
    echo "[ERROR] Failed to save map. Is SLAM Toolbox still running?"
fi
