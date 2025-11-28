import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lib_path = '/home/ubuntu/桌面/RM26/src/ros2_armor_can/lib/bin/unix64/release/'
    env = {'LD_LIBRARY_PATH': lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')}
    return LaunchDescription([
        Node(
            package='ros2_armor_can',
            executable='test_send_node',
            name='test_send_node',
            output='screen',
            additional_env=env
        ),
    ])