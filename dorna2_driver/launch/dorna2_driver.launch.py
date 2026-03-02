import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch the dorna2 driver node.
    Override params via the YAML config or command line:
      ros2 launch dorna2_driver dorna2_driver.launch.py
      --ros-args -p model:=dorna_2 -p host:=10.0.0.14
    """
    pkg_share = get_package_share_directory('dorna2_driver')
    config_file = os.path.join(pkg_share, 'config', 'dorna2_params.yaml')

    driver_node = Node(
        package='dorna2_driver',
        executable='dorna2_node',
        name='dorna2_driver',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([driver_node])
