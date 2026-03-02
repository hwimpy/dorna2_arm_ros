"""
Launch Gazebo simulation with the Dorna 2 robot.
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('dorna2_gazebo')
    desc_pkg = get_package_share_directory('dorna2_description')

    model_arg = DeclareLaunchArgument('model', default_value='dorna_ta')
    use_mesh_arg = DeclareLaunchArgument('use_mesh', default_value='true')

    xacro_file = os.path.join(gazebo_pkg, 'urdf', 'dorna2_gazebo.urdf.xacro')
    rviz_config = os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')

    robot_description = Command([
        'xacro ', xacro_file,
        ' model:=', LaunchConfiguration('model'),
        ' use_mesh:=', LaunchConfiguration('use_mesh'),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'dorna2',
            '-z', '0.0',
        ],
        output='screen',
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        model_arg,
        use_mesh_arg,
        gz_sim,
        robot_state_publisher,
        gz_spawn,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn,
                on_exit=[joint_trajectory_controller],
            )
        ),
        rviz,
    ])
