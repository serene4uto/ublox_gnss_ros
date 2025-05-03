from launch import LaunchDescription
from launch_ros.actions import Node


ARGS = [
    
]


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='ublox_gnss_ros',
            executable='ublox_gnss_node',
            name='ublox_gnss_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'frame_id': 'gps'},
            ],
            remappings=[
                ('/fix', '/ublox/fix'),
                ('/nmea', '/ublox/nmea'),
                ('/rtcm', '/ublox/rtcm'),
            ],
        ),
    ])