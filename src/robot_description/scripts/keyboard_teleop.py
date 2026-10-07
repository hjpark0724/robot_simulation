#!/usr/bin/env python3
"""Minimal keyboard teleoperation for the N1 Gazebo simulation."""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class KeyboardTeleop(Node):
    LINEAR_SPEED = 0.10
    ANGULAR_SPEED = 0.50

    def __init__(self):
        super().__init__('keyboard_teleop')
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_command(self, key: str):
        command = Twist()
        if key == 'w':
            command.linear.x = self.LINEAR_SPEED
        elif key == 'x':
            command.linear.x = -self.LINEAR_SPEED
        elif key == 'a':
            command.angular.z = self.ANGULAR_SPEED
        elif key == 'd':
            command.angular.z = -self.ANGULAR_SPEED
        elif key != 's':
            return
        self._publisher.publish(command)

    def run(self):
        if not sys.stdin.isatty():
            self.get_logger().error('Keyboard teleop requires an interactive terminal.')
            return

        settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Keyboard control: w=forward, x=reverse, '
                               'a=left, d=right, s=stop, Ctrl-C=exit')
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    self.publish_command(sys.stdin.read(1).lower())
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.publish_command('s')
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
