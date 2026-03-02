"""
Full bringup: robot description + driver + RViz.

The driver node loads typed parameters from its YAML config file. String-typed
launch arguments (model, host) are safe to forward directly. For the integer
and boolean parameters (port, auto_connect), we use an OpaqueFunction to
resolve the launch arguments at evaluation time and pass them with correct
Python types.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context):
    desc_pkg = get_package_share_directory('dorna2_description')
    driver_pkg = get_package_share_directory('dorna2_driver')

    model = context.launch_configurations['model']
    host = context.launch_configurations['host']
    port = int(context.launch_configurations['port'])
    auto_connect = context.launch_configurations['auto_connect'].lower() == 'true'
    use_mesh = context.launch_configurations.get('use_mesh', 'true')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'dorna2.urdf.xacro')
    rviz_config = os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')
    driver_config = os.path.join(driver_pkg, 'config', 'dorna2_params.yaml')

    robot_description = Command([
        'xacro ', xacro_file,
        ' model:=', model,
        ' use_mesh:=', use_mesh,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    driver_node = Node(
        package='dorna2_driver',
        executable='dorna2_node',
        name='dorna2_driver',
        output='screen',
        parameters=[
            driver_config,
            {
                'model': model,
                'host': host,
                'port': port,
                'auto_connect': auto_connect,
            },
        ],
        remappings=[
            ('~/joint_states', '/joint_states'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(
            context.launch_configurations.get('rviz', 'true')),
    )

    return [robot_state_publisher, driver_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='dorna_ta',
            description='Robot model: dorna_ta, dorna_2, dorna_2s'),
        DeclareLaunchArgument(
            'host', default_value='localhost',
            description='Robot controller IP or hostname'),
        DeclareLaunchArgument(
            'port', default_value='443',
            description='WebSocket port'),
        DeclareLaunchArgument(
            'auto_connect', default_value='false',
            description='Connect to robot on startup'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'use_mesh', default_value='true',
            description='Use STL meshes (false = cylinder primitives)'),
        OpaqueFunction(function=_launch_setup),
    ])
