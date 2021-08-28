from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros2',
            namespace='rplidar',
            executable='rplidarNode',
            name='rplidarNode_launch',
            output='screen'
         ),
    ])
