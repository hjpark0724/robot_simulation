#!/usr/bin/env python3
"""Fan out one gripper command to the six mechanically coupled joints."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperCommandRouter(Node):
    """Preserve the former URDF mimic multipliers without physics constraints."""

    # joint_name: multiplier relative to rlink1_Joint
    MULTIPLIERS = {
        'llink1': 1.0,
        'llink2': -1.0,
        'llink3': 1.0,
        'rlink1': 1.0,
        'rlink2': 1.0,
        'rlink3': -1.0,
    }

    def __init__(self):
        super().__init__('gripper_command_router')
        self._command_publishers = {
            joint: self.create_publisher(
                Float64, f'/gripper/{joint}/cmd_pos', 10)
            for joint in self.MULTIPLIERS
        }
        self.create_subscription(
            Float64, '/gripper/cmd_pos', self._command_callback, 10)

    def _command_callback(self, command: Float64):
        for joint, multiplier in self.MULTIPLIERS.items():
            target = Float64()
            target.data = multiplier * command.data
            self._command_publishers[joint].publish(target)


def main():
    rclpy.init()
    node = GripperCommandRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
