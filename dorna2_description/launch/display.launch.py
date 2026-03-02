import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('dorna2_description')

    model_arg = DeclareLaunchArgument(
        'model', default_value='dorna_ta',
        description='Robot model: dorna_ta, dorna_2, dorna_2s'
    )
    use_mesh_arg = DeclareLaunchArgument(
        'use_mesh', default_value='true',
        description='Use STL meshes for visualization (false = primitives)'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch joint_state_publisher_gui'
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'dorna2.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        model_arg,
        use_mesh_arg,
        gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
