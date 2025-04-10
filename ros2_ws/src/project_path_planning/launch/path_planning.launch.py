import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    planner_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'planner_server.yaml')
    controller_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'bt_navigator.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('project_path_planning'), 'config', 'recovery.yaml')

    nav2_planner = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]    
    )

    nav2_controller = Node(
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
            
    )

    nav2_bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
            
    )

    nav2_recovery_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'
    )
    nav2_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['planner_server','controller_server','bt_navigator','recoveries_server']}]
    )
    
    
    
    # Create the launch description and populate it with the nodes
    ld = LaunchDescription()

    ld.add_action(nav2_controller)
    ld.add_action(nav2_planner)
    ld.add_action(nav2_bt_navigator)
    ld.add_action(nav2_recovery_server)
    ld.add_action(nav2_lifecycle_node)

    return ld