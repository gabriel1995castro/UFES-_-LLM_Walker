from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vwalker_hri',
            executable='force_filter_node',
            name='force_filter',
            output='screen',
            parameters=[Path(get_package_share_directory('vwalker_hri'), 'param', 'force_filter_node.yaml')]
        ),
        Node(
            package='vwalker_hri',
            executable='virtual_trail_admittance_controller_node',
            name='virtual_trail_admittance_controller',
            output='screen',
            parameters=[Path(get_package_share_directory('vwalker_hri'), 'param', 'admittance_controller_node.yaml')]
        ),
        Node(
            package='vwalker_hri',
            executable='superviser_node',
            name='superviser',
            output='screen',
            parameters=[Path(get_package_share_directory('vwalker_hri'), 'param', 'superviser_node.yaml')]
        ),
        
        Node(
            package='vwalker_hri',
            executable='path_following',
            name='path_following',
            output='screen',
        )



    ])