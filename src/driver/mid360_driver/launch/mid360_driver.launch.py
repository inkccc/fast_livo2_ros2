import launch
import launch_ros
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml



def generate_launch_description():
    package_path = get_package_share_directory('mid360_driver')

    declare_use_robot_description = launch.actions.DeclareLaunchArgument(
        'use_robot_description',
        default_value='false',
        description='Whether to publish robot_description'
    )
    use_robot_description = LaunchConfiguration('use_robot_description')
    urdf_file_path = os.path.join(package_path, 'urdf', 'robot.xacro')
    declare_use_rviz = launch.actions.DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file_path = os.path.join(package_path, 'config', 'rviz', 'mid360_driver.rviz')


    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path]),
                value_type=str
            )
        }],
        condition=launch.conditions.IfCondition(use_robot_description)
    )

    action_rviz2 = launch.actions.ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file_path],
        condition=launch.conditions.IfCondition(use_rviz),
        output='screen'
    )
    
    # livoxmid360 驱动节点
    pkg_path_mid360_driver = get_package_share_directory('mid360_driver')
    #/home/inkc/inkc/rws/src/driver/mid360_driver/config/param.yaml
    config_path_mid360_driver = os.path.join(pkg_path_mid360_driver, 'config', 'param.yaml')
    action_mid360_driver = launch_ros.actions.Node(
        package='mid360_driver',  # 包名
        executable='mid360_driver_node',  # 可执行文件名
        name='mid360_driver',  # 节点名称
        output='screen',  # 输出到屏幕
        parameters=[config_path_mid360_driver]  # 配置文件路径
    )
    print("mid360_driver config path:", config_path_mid360_driver)

    return LaunchDescription([
        declare_use_robot_description,
        declare_use_rviz,
        action_mid360_driver,
        action_robot_state_publisher,
        action_rviz2,
    ])
