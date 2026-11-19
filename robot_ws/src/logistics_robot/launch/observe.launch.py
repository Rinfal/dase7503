from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_directory("logistics_robot") + "/config/robot.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(package="logistics_robot", executable="safety_gate", name="safety_gate",
             parameters=[LaunchConfiguration("config")], output="screen"),
        Node(package="logistics_robot", executable="odom_tf", name="odom_tf",
             parameters=[LaunchConfiguration("config")], output="screen"),
        Node(package="logistics_robot", executable="qr_scanner", name="qr_scanner",
             parameters=[LaunchConfiguration("config")], output="screen"),
    ])
