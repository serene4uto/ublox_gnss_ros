from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("ublox_gnss")
    mapviz_config_file = os.path.join(gps_wpf_dir, "config", "gps_wpf_demo.mvc")

    node_mapviz = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[{"config": mapviz_config_file}]
    )


    ld = LaunchDescription()

    ld.add_action(node_mapviz)

    return ld