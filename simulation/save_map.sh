#!/bin/bash

# ==========================================
# LK1209 地图保存工具
# ROS2 Humble + Nav2
# ==========================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAP_DIR="$ROOT/ros2_ws/src/robot_navigation/maps"

MAP_NAME="${1:-lk1209_map}"

MAP_PREFIX="$MAP_DIR/$MAP_NAME"

mkdir -p "$MAP_DIR"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

echo "======================================"
echo "       LK1209 地图保存工具"
echo "======================================"

echo "[INFO] 地图名称：$MAP_NAME"
echo "[INFO] 保存目录：$MAP_DIR"

# 检查地图话题是否存在
if ! ros2 topic list | grep -qx "/map"; then
    echo "[ERROR] 没有发现 /map 话题"
    echo "[ERROR] 请先启动 SLAM 建图"
    exit 1
fi

echo "[INFO] 正在保存地图..."

ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_PREFIX" \
    --ros-args -p use_sim_time:=true

echo "======================================"
echo "[SUCCESS] 地图保存完成"
echo "======================================"

echo "$MAP_PREFIX.yaml"
echo "$MAP_PREFIX.pgm"

