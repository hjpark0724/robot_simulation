#!/usr/bin/env python3
"""Rewrite the bridged IMU message's frame_id to match the URDF TF frame.

ros_gz_bridge publishes the Gazebo IMU sensor's scoped name (e.g.
``n1_robot/base_footprint/imu_sensor``) as ``header.frame_id``, but the URDF
TF tree names that frame ``imu_frame``. This node subscribes to the raw
bridged topic and republishes the same message with the frame_id corrected,
so consumers that look up ``imu_frame`` in TF get a matching sensor frame.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFrameRelay(Node):
    def __init__(self):
        super().__init__('imu_frame_relay')
        self.declare_parameter('source_topic', '/imu/raw')
        self.declare_parameter('target_topic', '/imu')
        self.declare_parameter('target_frame', 'imu_frame')

        source_topic = self.get_parameter('source_topic').value
        target_topic = self.get_parameter('target_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._publisher = self.create_publisher(Imu, target_topic, 10)
        self.create_subscription(Imu, source_topic, self._imu_callback, 10)

    def _imu_callback(self, message: Imu):
        message.header.frame_id = self._target_frame
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = ImuFrameRelay()
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
