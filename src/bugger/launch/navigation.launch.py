import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = get_package_share_directory('bugger')
    nav2_params_path = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    amcl_params_path = os.path.join(nav_dir, 'config', 'amcl_params.yaml')
    
    
    map_server_node = Node(                                 ## Map Server
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(nav_dir, 'maps', 'cafe.yaml'),
             'use_sim_time': True}]
    )

    
    amcl_node = Node(                                       ## AMCL( Localization)
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_path, {'use_sim_time': True}]
    )

    
    controller_server = Node(                               ## Controller Server
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': True}]
    )

    
    planner_server = Node(                                  ## Planner Server
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': True}]
    )

   
    behaviors_server = Node(                                 # # behaviors Server
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': True}],
        remappings=[
        ('/cmd_vel', '/cmd_vel_behavior')
        ]

    )

    
    bt_navigator = Node(                                    ## BT Navigator
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': True}]
    )

    
    lifecycle_manager_node = Node(                          ## Lifecycle Manager (Handles the  activation )
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 
                                   'amcl', 
                                   'controller_server', 
                                   'planner_server', 
                                   'behavior_server', 
                                   'bt_navigator']}]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        controller_server,
        planner_server,
        behaviors_server,
        bt_navigator,
        lifecycle_manager_node
    ])