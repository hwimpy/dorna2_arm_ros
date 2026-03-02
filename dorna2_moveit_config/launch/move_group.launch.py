"""
MoveIt 2 move_group launch for Dorna 2 series.
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def _launch_setup(context):
    model = context.launch_configurations['model']
    use_mesh = context.launch_configurations.get('use_mesh', 'true')

    moveit_pkg = get_package_share_directory('dorna2_moveit_config')
    desc_pkg = get_package_share_directory('dorna2_description')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'dorna2.urdf.xacro')
    robot_description = Command([
        'xacro ', xacro_file,
        ' model:=', model,
        ' use_mesh:=', use_mesh,
    ])

    srdf_xacro = os.path.join(moveit_pkg, 'config', 'dorna2.srdf.xacro')
    robot_description_semantic = Command([
        'xacro ', srdf_xacro,
        ' model:=', model,
    ])

    kinematics_yaml = _load_yaml('dorna2_moveit_config', 'config/kinematics.yaml')
    ompl_yaml = _load_yaml('dorna2_moveit_config', 'config/ompl_planning.yaml')
    joint_limits_yaml = _load_yaml('dorna2_moveit_config', 'config/joint_limits.yaml')

    ctrl_suffix = '6dof' if model == 'dorna_ta' else '5dof'
    controllers_yaml = _load_yaml(
        'dorna2_moveit_config', f'config/moveit_controllers_{ctrl_suffix}.yaml')

    move_group_params = {
        'robot_description': ParameterValue(robot_description, value_type=str),
        'robot_description_semantic': ParameterValue(robot_description_semantic, value_type=str),
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,
    }
    move_group_params.update(controllers_yaml)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    rviz_config = os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            {'robot_description_semantic': ParameterValue(robot_description_semantic, value_type=str)},
            {'robot_description_kinematics': kinematics_yaml},
        ],
    )

    return [robot_state_publisher, move_group_node, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='dorna_ta'),
        DeclareLaunchArgument('use_mesh', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
