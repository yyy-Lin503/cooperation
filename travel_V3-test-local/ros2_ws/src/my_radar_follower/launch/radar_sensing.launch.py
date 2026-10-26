from my_radar_follower.runtime_support import runtime_paths
"""No command publisher; usable for stationary data collection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable,LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic',default_value='/odom'),
        DeclareLaunchArgument('audit_dir',default_value=str(runtime_paths()['track'])),
        Node(package='my_radar_follower',executable='radar_odom_receiver',name='radar_odom_receiver',output='screen',parameters=[
            {'odom_topic':LaunchConfiguration('odom_topic'),'audit_dir':LaunchConfiguration('audit_dir')}])])
