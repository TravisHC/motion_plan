from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_processing',
            executable='path_processing_node',
            name='path_processing_node',
            output='screen',
            parameters=['config/path_processing_params.yaml']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', 'config/path_processing.rviz']
        )
    ])