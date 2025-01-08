from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='propeller_control',
            executable='propeller_control',
            parameters=[PathJoinSubstitution([FindPackageShare('propeller_control'), 'config', 'pid_param.yaml'])],
            output='screen',
            emulate_tty=True,
        ),
    ])