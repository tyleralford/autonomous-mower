import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='mower_description').find('mower_description')
    
    # Default paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'mower.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')
    
    # Launch configuration variables
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    
    return LaunchDescription([
        # Fix snap conflicts
        SetEnvironmentVariable('GTK_PATH', ''),
        SetEnvironmentVariable('GIO_MODULE_DIR', ''),
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='Start joint state publisher GUI (disable when using ros2_control)'
        ),
        
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', model]), value_type=str
                ),
                'use_sim_time': use_sim_time,
                'publish_frequency': 200.0  # Match controller update rate
            }]
        ),
        
        # Joint State Publisher GUI Node (conditional)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_joint_state_gui)
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
