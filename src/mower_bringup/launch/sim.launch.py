import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_mower_bringup = FindPackageShare(package='mower_bringup').find('mower_bringup')
    pkg_mower_description = FindPackageShare(package='mower_description').find('mower_description')
    pkg_mower_simulation = FindPackageShare(package='mower_simulation').find('mower_simulation')
    pkg_mower_control = FindPackageShare(package='mower_control').find('mower_control')
    pkg_mower_localization = FindPackageShare(package='mower_localization').find('mower_localization')
    
    # World file path
    world_file = os.path.join(pkg_mower_simulation, 'worlds', 'lawn.world')
    
    # Controller config file path  
    controller_config = os.path.join(pkg_mower_control, 'config', 'mower_controllers.yaml')

    # Localization config files
    ekf_local_config = os.path.join(pkg_mower_localization, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_mower_localization, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_mower_localization, 'config', 'navsat_transform.yaml')
    
    # Nridge config file
    gz_bridge_config = os.path.join(pkg_mower_bringup, 'config', 'gz_bridge.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    return LaunchDescription([
        # Fix snap conflicts
        SetEnvironmentVariable('GTK_PATH', ''),
        SetEnvironmentVariable('GIO_MODULE_DIR', ''),
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for visualization'
        ),
        
        # Include the robot description display launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mower_description'),
                    'launch',
                    'display.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_joint_state_gui': 'false',  # Disable GUI since we use ros2_control
                # Use the EKF test RViz configuration for loop-closure validation
                'rviz_config': os.path.join(pkg_mower_localization, 'rviz', 'ekf_test.rviz')
            }.items()
        ),
        
        # Launch Gazebo with the specified world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': [world_file, ' -r -v 1'],
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Consolidated ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[
                {'config_file': gz_bridge_config},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Ground Truth Heading Node - Calculates absolute heading from ground truth with noise
        Node(
            package='mower_localization',
            executable='ground_truth_heading_node',
            name='ground_truth_heading_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Local EKF - fuses wheel odom and IMU, publishes odom->base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf_node',
            output='screen',
            parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/local')
            ]
        ),

        # NavSat Transform Node - converts GPS to map-frame odometry
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/imu', '/gps/heading'),
                ('/gps/fix', '/gps/fix'),
                ('/odometry/filtered', '/odometry/filtered/local'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),

        # Global EKF - fuses local EKF, GPS position, and GPS heading, publishes map->odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/global')
            ]
        ),

        # Recorder node for drive-to-record zone creation service
        Node(
            package='mower_localization',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Spawn the robot using the gz_spawn_model launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_spawn_model.launch.py'
                ])
            ]),
            launch_arguments={
                'name': 'mower',
                'topic': 'robot_description',
                'x': '0.0',
                'y': '0.0',
                'z': '0.2'
            }.items()
        ),
        
        # Note: Controller manager is provided by the Gazebo gz_ros2_control plugin
        # No need for a standalone ros2_control_node

        # Joint State Broadcaster (replaces joint_state_publisher_gui)
        # Delay spawning to ensure Gazebo controller manager is ready
        TimerAction(period=2.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true'
                ],
                output='screen'
            )
        ]),

        # Differential Drive Controller for chassis movement
        TimerAction(period=3.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true'
                ],
                output='screen'
            )
        ]),

        # Reel Controller for cutting reel
        TimerAction(period=3.5, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'reel_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-ros-args', '--ros-args -p use_sim_time:=true'
                ],
                output='screen'
            )
        ])
    ])
