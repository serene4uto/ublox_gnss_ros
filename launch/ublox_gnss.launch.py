from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGS = [
    
]

def generate_launch_description():
    
    gnss_config = PathJoinSubstitution(
        [FindPackageShare('ublox_gnss_ros'), 'config', 'gnss.yaml']
    )
    
    return LaunchDescription(ARGS + [
        Node(
            package='ublox_gnss_ros',
            executable='ublox_gnss_node',
            name='ublox_gnss_node',
            output='screen',
            parameters=[gnss_config],
            remappings=[
                ('/fix', '/ublox/fix'),
                ('/nmea', '/ublox/nmea'),
                ('/rtcm', '/ublox/rtcm'),
            ],
        ),
    ])