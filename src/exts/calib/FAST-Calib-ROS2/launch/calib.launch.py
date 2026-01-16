from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Get package share directory
    pkg_share = FindPackageShare('fast_calib')
    
    # Load parameters
    params_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])
    
    # Fast calibration node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[params_file]
    )
    
    # RViz node
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        fast_calib_node,
        rviz_node
    ]) 	
