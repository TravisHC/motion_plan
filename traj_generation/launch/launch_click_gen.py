from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 traj_generation 包的路径
    traj_generation_dir = get_package_share_directory('traj_generation')
    config_dir = os.path.join(traj_generation_dir, 'config')
    # 构建参数文件路径
    # yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(traj_generation_dir, 'config', 'click_gen.yaml'),
    #     description='Full path to map file to load')
    # rviz_cmd = DeclareLaunchArgument(
    #     'rviz',
    #     default_value=os.path.join(traj_generation_dir, 'config', 'click_gen.rviz'),
    #     description='Full path to rviz file to load')
    # param_file_path = os.path.join(traj_generation_dir, 'config', 'click_gen.yaml')
    rviz_file_path = os.path.join(traj_generation_dir, 'config', 'click_gen.rviz')

    return LaunchDescription([
        Node(
            package='traj_generation',
            executable='click_gen',
            # parameters=[param_file_path]  # 加载参数文件
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file_path]  # 加载rviz文件
        )
    ])