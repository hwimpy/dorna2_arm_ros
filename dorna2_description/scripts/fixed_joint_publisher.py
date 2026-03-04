#!/usr/bin/env python3
"""Publish a fixed joint state at 10 Hz with proper timestamps.

Usage (6-DOF / Dorna TA):
    fixed_joint_publisher.py <j0> <j1> <j2> <j3> <j4> <j5>

Usage (5-DOF / Dorna 2, 2S):
    fixed_joint_publisher.py <j0> <j1> <j2> <j3> <j4>

Joint positions are in radians. The number of arguments determines
whether 5-DOF or 6-DOF joint names are published.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES_6DOF = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
JOINT_NAMES_5DOF = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4']


class FixedJointPublisher(Node):
    def __init__(self, joint_names, positions):
        super().__init__('fixed_joint_publisher')
        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._msg = JointState()
        self._msg.name = joint_names
        self._msg.position = positions
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f'Publishing fixed pose ({len(joint_names)} DOF): {positions}')

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main():
    args = sys.argv[1:]
    if len(args) == 5:
        joint_names = JOINT_NAMES_5DOF
    elif len(args) == 6:
        joint_names = JOINT_NAMES_6DOF
    else:
        print(f'Usage: {sys.argv[0]} j0 j1 j2 j3 j4 [j5]', file=sys.stderr)
        print('  5 args -> Dorna 2/2S (5-DOF)', file=sys.stderr)
        print('  6 args -> Dorna TA   (6-DOF)', file=sys.stderr)
        sys.exit(1)

    positions = [float(x) for x in args]
    rclpy.init()
    node = FixedJointPublisher(joint_names, positions)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
