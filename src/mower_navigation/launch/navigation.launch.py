from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='/home/tyler/mower_ws/src/mower_navigation/config/nav2_params.yaml'),

        # Map server - loads the generated map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file]
        ),
        
        # Planner server - path planning using SmacPlannerHybrid
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file]
        ),
        
        # Controller server - path following using DWB controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            remappings=[
                # Use GPS-corrected odometry instead of AMCL
                ('/odom', '/odometry/filtered/global')
            ]
        ),
        
        # Behavior Tree Navigator - high-level navigation logic
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            remappings=[
                # Use GPS-corrected odometry instead of AMCL
                ('/odom', '/odometry/filtered/global')
            ]
        ),
        
        # Behavior server - recovery behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file]
        ),
        
        # Lifecycle manager - manages Nav2 node lifecycle
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'behavior_server']
            }] 
        )
    ])
