from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_ros',
            executable='gps_pub',
            name='gps_pub',
        ),

        Node(
            package='ntrip_ros',
            executable='rtcm_pub_node',
            name='rtcm_pub_node',
        )
    ])