from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime

def generate_launch_description():
    current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_dir = os.path.expanduser(f'~/ros2_ws/bags/{current_time}')
    
    pkg_share = get_package_share_directory('line_follower')
    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Record the data'
    )

    record_config = LaunchConfiguration('record')

    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir, '--topics', '/error', '/cmd_vel', '/detections', '/rosout', '/traffic_light'],
        output='screen',
        condition=IfCondition(record_config) 
    )
    
    return LaunchDescription([
        record_arg,
        Node(
            package='line_follower',
            executable='camera_node',
            parameters=[param_file],
        ),
        Node(
            package='line_follower',
            executable='line_detection_node',
            parameters=[param_file],
        ),
        Node(
            package='line_follower',
            executable='detection_node',
            parameters=[param_file],
        ),
        Node(
            package='line_follower',
            executable='pid_node',
            parameters=[param_file],
        ),
        Node(
            package='line_follower',
            executable='serial_node',
            parameters=[param_file],
        ),
        Node(
            package='line_follower',
            executable='traffic_light_node'
        ),
        Node(
            package='line_follower',
            executable='state_machine_node',
            parameters=[param_file],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        record_process,
    ])