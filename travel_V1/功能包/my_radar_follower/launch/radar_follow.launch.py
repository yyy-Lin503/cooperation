from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track_dir', default_value=EnvironmentVariable(
            'RADAR_TRACK_DIR', default_value=str(Path.cwd() / 'track_txt'))),
        DeclareLaunchArgument('max_linear_speed_mps', default_value='0.8'),
        DeclareLaunchArgument('max_angular_speed_radps', default_value='0.8'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        # Odom receiver is started separately by radar_sensing.launch.py.
        Node(
            package='my_radar_follower',
            executable='txt_radar_bridge',
            name='txt_radar_bridge',
            output='screen',
            parameters=[
                str(Path(get_package_share_directory('my_radar_follower')) /
                    'config' / 'bridge.yaml'),
                {
                    'track_dir': LaunchConfiguration('track_dir'),
                    'max_linear_speed_mps': LaunchConfiguration('max_linear_speed_mps'),
                    'max_angular_speed_radps': LaunchConfiguration('max_angular_speed_radps'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                },
            ],
        )
    ])
