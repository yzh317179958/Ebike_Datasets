# ebike_human_follower - 电动自行车行人跟随系统

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-brightgreen)](https://www.ros.org/)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

## 1. 系统依赖
### 1.1 **系统环境及传感器驱动**

**Ubuntu == 22.04** 

**ROS = Humble** [ROS Installation](https://blog.csdn.net/m0_73745340/article/details/135281023)

Lidar. [rslidar_SDK]()
### 1.2 **FAST-LIO**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).necessary!

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page). necessary!

livox_ros_driver[livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver). not useful but necessary！

### 1.3 **humanpose（达杰）**

待更新...

### 1.4 **pointprocessing（素慧）**

待更新...

## 2. Build
克隆仓库代码并编译

```bash
    cd <ros2_ws>/src # 进入工作空间下的src目录
    git clone https://github.com/yzh317179958/ebike_human_follower.git --recursive
    cd ..
    colcon build --symlink-install
    . ./install/setup.bash 
```

****
## 3. 运行指令

```bash
    cd<ros2_ws>
    source install/setup.bash
    ./start.sh
```

## 4. 传感器话题数据
雷达话题为/rslidar_points. IMU发布的话题为/imu/data_raw. 

---

## 系统架构图

```mermaid
graph TD
    %% 感知层
    subgraph 感知层
        A[IMU H30] -->|200Hz /imu/data_raw| F[FAST-LIO]
        B[LiDAR E1R] -->|10Hz /rslidar_points| F
        C[Camera Gemini] -->|30Hz RGB-D| G[人体检测]
        F -->|10Hz odom| H[Odom融合]
        G -->|30Hz bbox| I[3D定位]
        B -->|10Hz rslidar_points| I
        F -->|10Hz map| J[障碍物地图]
    end
    
    %% 数据总线
    H -->|/odom| K[(Odom)]
    I -->|/human_pose| L[(Pose)]
    J -->|/obstacle_map| M[(Map)]
    
    %% 决策层
    subgraph 决策层
        K --> N[路径规划]
        L --> N
        M --> N
        N -->|/path| O[运动控制]
        L -->|/track| O
        K --> O
    end
    
    %% 执行层
    subgraph 执行层
        O -->|/cmd_vel| P[电机控制]
        O -->|/steer| Q[转向控制]
        O -->|/brake| R[制动系统]
    end
    
    %% 交互层
    subgraph 交互层
        S[Web UI] -->|/params| T[ROS Param]
        U[Mobile App] -->|/cmd| V[Command]
        W[Voice] -->|/voice| V
        V --> O
    end
    
    %% 数据存储
    X[Logger] -.->|/rosbag| Y[(Bag)]


