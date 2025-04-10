import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path where the map will be saved
    map_file_path = os.path.expanduser('~/ros2_ws/src/project_mapping/maps/map')

    # Ensure the directory exists
    os.makedirs(os.path.dirname(map_file_path), exist_ok=True)

    # Map saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_file_path],  # Use the -f argument to specify the file path
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(map_saver_node)

    return ld