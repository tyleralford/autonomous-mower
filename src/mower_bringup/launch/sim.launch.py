import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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
    
    # World file path
    world_file = os.path.join(pkg_mower_simulation, 'worlds', 'lawn.world')
    
    # Controller config file path  
    controller_config = os.path.join(pkg_mower_control, 'config', 'mower_controllers.yaml')
    
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
                'use_joint_state_gui': 'false'  # Disable GUI since we use ros2_control
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
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster', 
                '--controller-manager', '/controller_manager',
                '--controller-ros-args', '--ros-args --remap /clock:=/clock -p use_sim_time:=true'
            ],
            output='screen'
        ),

        # Differential Drive Controller for chassis movement
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller', 
                '--controller-manager', '/controller_manager',
                '--controller-ros-args', '--ros-args --remap /clock:=/clock -p use_sim_time:=true'
            ],
            output='screen'
        ),

        # Reel Controller for cutting reel
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'reel_controller', 
                '--controller-manager', '/controller_manager',
                '--controller-ros-args', '--ros-args --remap /clock:=/clock -p use_sim_time:=true'
            ],
            output='screen'
        ),
    ])
