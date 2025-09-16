from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 获取工作空间路径
    workspace_dir = os.path.expanduser('~/Ebike_Human_Follower')
    model_path = os.path.join(workspace_dir, 'src/posetrack/models/yolov8n_pose_bayese_640x640_nv12_modified.bin')
    osnet_model_path = os.path.join(workspace_dir, 'src/posetrack/models/osnet_64x128_nv12.bin')

    # 1. RS-LiDAR 雷达启动
    rslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rslidar_sdk'),
                'launch',
                'start.py'
            ])
        ])
    )
    
    # 2. Yesense IMU 启动
    yesense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yesense_std_ros2'),
                'launch',
                'yesense_node.launch.py'
            ])
        ])
    )
    
    # 3. Orbbec 相机启动
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py'
            ])
        ]),
        launch_arguments={
            'color_width': '640',
            'color_height': '480',
            'color_fps': '10',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '10',
            'depth_registration': 'true'
        }.items()
    )
    
    # 4. fast_lio launch 文件
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': 'my2.yaml',
            'use_sim_time': 'false'
        }.items()
    )
    
    # 5. 地面分割节点
    ground_segmentation_node = Node(
        package='ground_segmentation',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen'
    )
    
    # 6. 点云到占据栅格节点
    pointcloud_to_grid_node = Node(
        package='pointcloud_to_grid',
        executable='pointcloud_to_occupancy_grid_node',
        name='pointcloud_to_grid',
        output='screen'
    )
    
    # 7. 组合导航节点
    integrated_navigation_node = Node(
        package='integrated_navigation',
        executable='integrated_navigation_node',
        name='integrated_navigation',
        output='screen'
    )
    
    # 8. YOLOv11 姿态跟踪节点
    yolov11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov11_pose_tracking'),
                'launch',
                'yolov11_pose_tracking.launch.py'
            ])
        ]),
    )
    
    # 9. 自行车控制节点
    bike_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                workspace_dir,
                'install/bike_control/share/bike_control/bike_control.launch.py'
            )
        ])
    )


    # 设置启动顺序
    delayed_yesense = TimerAction(period=3.0, actions=[yesense_launch])
    delayed_orbbec = TimerAction(period=6.0, actions=[orbbec_launch])
    delayed_fast_lio = TimerAction(period=10.0, actions=[fast_lio_launch])
    delayed_ground_segmentation = TimerAction(period=12.0, actions=[ground_segmentation_node])
    delayed_pointcloud_grid = TimerAction(period=14.0, actions=[pointcloud_to_grid_node])
    delayed_yolov11 = TimerAction(period=16.0, actions=[yolov11_launch])
    delayed_integrated_nav = TimerAction(period=20.0, actions=[integrated_navigation_node])
    delayed_bike_control = TimerAction(period=22.0, actions=[bike_control_launch])

    return LaunchDescription([

        # 首先启动传感器
        rslidar_launch,
        
        # 延迟启动其他节点
        delayed_yesense,
        delayed_orbbec,
        delayed_fast_lio,
        delayed_ground_segmentation,
        delayed_pointcloud_grid,
        delayed_yolov11,
        delayed_integrated_nav,
        delayed_bike_control
    ])
