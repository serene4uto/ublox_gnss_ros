# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='ublox_gnss',
#             executable='gnss_pub',
#             name='gnss_pub',
#         ),

#         Node(
#             package='rtcm_provider',
#             executable='rtcm_ntrip_pub',
#             name='rtcm_ntrip_pub',
#         )
#     ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'ublox_gnss', 'gnss_pub'],
            name='gnss_pub',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rtcm_provider', 'rtcm_ntrip_pub'],
            name='rtcm_ntrip_pub',
        )
    ])