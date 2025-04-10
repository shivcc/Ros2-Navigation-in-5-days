from launch import LaunchDescription
from launch_ros.actions import Node
import os

map_file_path = os.path.expanduser('~/ros2_ws/src/project_mapping/maps/map.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'autostart': True},
                         {'node_names': ['map_server']}]
        )
    ])