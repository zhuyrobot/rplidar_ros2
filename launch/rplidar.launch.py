from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rplidar_ros2',
            namespace='',
            executable='rplidarNode',
            name='rplidarNode_launch',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
         ),
    ])
