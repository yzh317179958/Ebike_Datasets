#!/bin/bash
ros2 bag record \
    /centroid_point \
    /tracks \
    /bike_control/visualization \
    /visualization_marker \
    /path \
    /user_position \
    /tf_static \
    /integrated_navigation_data \
    /camera/color/camera_info \
    /tf \
    /person_positions \
    /pointcloud_occupancy_grid \
    /Odometry \
    /rslidar_points \
    /imu/data_raw \
    /camera/color/image_raw \
    /camera/depth/image_raw \
    -s mcap \
    --max-cache-size 2147483648 

