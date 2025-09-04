from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    boundary_buffer_m = LaunchConfiguration('boundary_buffer_m')
    auto_start = LaunchConfiguration('auto_start')
    verbose = LaunchConfiguration('verbose')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='/home/tyler/mower_ws/src/mower_navigation/config/nav2_params.yaml'),
        # Exposed guard parameters
        DeclareLaunchArgument('boundary_buffer_m', default_value='0.2'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),

        # Guard node: waits for valid map and in-bounds pose, then starts Nav2 lifecycle
        Node(
            package='mower_navigation',
            executable='map_bounds_guard.py',
            name='map_bounds_guard',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'boundary_buffer_m': boundary_buffer_m,
                 'verbose': verbose,
                 'auto_start': auto_start,
                 # Keeping lifecycle schedule static (array typing issues via launch substitutions)
                 'lifecycle_retry_seconds': [0.0, 5.0, 10.0]}
            ]
        ),
        
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
        
        # Controller server - path following using DWB controller (map/odom frames)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            remappings=[('/odom', '/odometry/filtered')]
        ),

        # Velocity stamping bridge: converts Nav2 Twist -> TwistStamped for diff_drive_controller
        Node(
            package='mower_navigation',
            executable='cmd_vel_stamper.py',
            name='cmd_vel_stamper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Behavior Tree Navigator - high-level navigation logic (map frame)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            remappings=[('/odom', '/odometry/filtered')]
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
            parameters=[{'use_sim_time': use_sim_time}, params_file]
        )
    ])
