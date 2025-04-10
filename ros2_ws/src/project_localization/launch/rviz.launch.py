import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Define the configuration directory and basename
    
    rviz_file = os.path.join(get_package_share_directory('project_localization'),'config','config', 'localization.rviz')
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
    )
            
  

    # Create the launch description and populate it with the nodes
    ld = LaunchDescription()
    ld.add_action(rviz_node)
    return ld