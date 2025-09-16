#!/bin/bash

# 多传感器一键启动简化脚本
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="/home/sunrise/Ebike_Human_Follower"

# 初始化环境
source /opt/ros/$ROS_DISTRO/setup.bash
source $WORKSPACE_DIR/install/setup.bash

# 启动各节点
taskset -c 0 ros2 launch rslidar_sdk start.py &
sleep 2

taskset -c 1 ros2 launch yesense_std_ros2 yesense_node.launch.py &
sleep 2

taskset -c 2 ros2 launch orbbec_camera gemini_330_series.launch.py \
  color_weight:=640 color_height:=480 color_fps:=10 \
  depth_weight:=640 depth_height:=480 depth_fps:=10 depth_registration:=false &
sleep 2

# 捕获终止信号
trap "kill $(jobs -p) 2>/dev/null; exit" SIGINT SIGTERM

wait
