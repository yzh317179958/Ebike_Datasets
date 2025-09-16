from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取工作空间根目录
    pkg_share = get_package_share_directory('bike_control')
    # 正确计算工作空间根目录
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    
    # 构建配置文件路径
    config_path = os.path.join(workspace_dir, 'config', 'bike_config.yaml')
    
    # 打印路径用于调试
    print(f"配置文件路径: {config_path}")
    
    # 检查文件是否存在
    if not os.path.isfile(config_path):
        print(f"错误: 配置文件不存在: {config_path}")
        # 使用默认配置文件作为备选
        default_config = os.path.join(pkg_share, 'config', 'bike_config.yaml')
        if os.path.isfile(default_config):
            print(f"使用默认配置文件: {default_config}")
            config_path = default_config
    
    return LaunchDescription([
        Node(
            package='bike_control',
            executable='ebike_control',
            name='ebike_control_node',
            parameters=[config_path],
            output='screen'
        )
    ])
