#!/bin/bash

# 多传感器启动简化脚本
WORKSPACE_DIR="/home/sunrise/Ebike_Human_Follower"


# 初始化环境
source /opt/ros/humble/setup.bash
source $WORKSPACE_DIR/install/setup.bash

# 启动所有节点
ros2 launch fast_lio mapping.launch.py config_file:=my2.yaml &
sleep 3

ros2 run ground_segmentation ground_segmentation_node &
#ros2 run ground_segmentation ground_segmentation_node &
sleep 3

ros2 run pointcloud_to_grid pointcloud_to_occupancy_grid_node &
sleep 3

ros2 launch yolov11_pose_tracking yolov11_pose_tracking.launch.py
sleep 3

#ros2 run integrated_navigation integrated_navigation_node
#sleep 3


#ros2 launch bike_control bike_control.launch.py
#ros2 run bike_control ebike_control
sleep 3

# 返回工作目录
cd $WORKSPACE_DIR

# 保持运行
echo "系统已启动，按Ctrl+C退出"
wait
