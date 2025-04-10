import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Define the configuration directory and basename
    map_file = os.path.expanduser('~/ros2_ws/src/project_mapping/maps/map.yaml')
    amcl_yaml = os.path.join(get_package_share_directory('project_localization'),'config','config', 'amcl.yaml')

    # Define the nodes

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} 
                       ]
    )
    
    amcl_server = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True},
                         {'node_names': ['map_server', 'amcl']}]
    )

    set_init_pose = Node(
            package='project_localization',
            executable='set_init_pose.py',
            name='set_init_pose',
            output='screen'
    )

    # Create the launch description and populate it with the nodes
    ld = LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(amcl_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(set_init_pose)


    return ld