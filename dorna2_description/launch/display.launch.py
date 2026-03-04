import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _joint_source(context):
    """Launch either a fixed-pose publisher (when 'pose' is set) or the
    default joint_state_publisher (when headless without a pose)."""
    pose_str = LaunchConfiguration('pose').perform(context)
    gui_str = LaunchConfiguration('gui').perform(context)

    if gui_str.lower() == 'true':
        return []

    if pose_str:
        joints = pose_str.split(',')
        script_path = os.path.join(
            get_package_share_directory('dorna2_description'),
            '..', '..', 'lib', 'dorna2_description', 'fixed_joint_publisher.py')
        return [ExecuteProcess(
            cmd=['python3', script_path] + joints,
            output='screen',
        )]

    return [Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )]


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
        description='Launch joint_state_publisher_gui and RViz (set false for headless/Docker)'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='false',
        description='Launch foxglove_bridge for Foxglove Studio visualization'
    )
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='WebSocket port for foxglove_bridge'
    )
    pose_arg = DeclareLaunchArgument(
        'pose', default_value='',
        description='Fixed joint pose as comma-separated values (e.g. "0,1.57,-1.57,0,0,0"). '
                    'When set, overrides joint_state_publisher.'
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
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'capabilities': ['clientPublish', 'assets'],
            'asset_uri_allowlist': [r'package://.*'],
        }],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription([
        model_arg,
        use_mesh_arg,
        gui_arg,
        foxglove_arg,
        foxglove_port_arg,
        pose_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        OpaqueFunction(function=_joint_source),
        rviz,
        foxglove_bridge,
    ])
