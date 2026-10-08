#!/usr/bin/env python3
"""Convert Gazebo RGB-D PointCloud2 coordinates to the ROS optical convention."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class GazeboPointCloudToOptical(Node):
    """Republish Gazebo's +X-forward RGB-D cloud as a +Z-forward ROS cloud."""

    def __init__(self):
        super().__init__('gz_pointcloud_to_optical')
        self.declare_parameter('source_topic', '/dcw/rgbd/points_raw')
        self.declare_parameter('target_topic', '/dcw/rgbd/points')
        self.declare_parameter('target_frame', 'dcw_depth_optical_frame')

        source_topic = self.get_parameter('source_topic').value
        target_topic = self.get_parameter('target_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._publisher = self.create_publisher(PointCloud2, target_topic, 10)
        self._subscription = self.create_subscription(
            PointCloud2, source_topic, self._pointcloud_callback, 10)

    def _pointcloud_callback(self, message: PointCloud2):
        fields = {field.name: field for field in message.fields}
        required = {'x', 'y', 'z'}
        if not required.issubset(fields):
            self.get_logger().error('Point cloud does not contain x, y, z fields.')
            return

        if any(fields[name].datatype != PointField.FLOAT32 for name in required):
            self.get_logger().error('Only FLOAT32 x, y, z point fields are supported.')
            return

        # View the raw byte buffer as one row per point, point_step bytes wide,
        # so every field (including any trailing RGB payload) rides along
        # untouched while only the x/y/z columns are rewritten in place.
        dtype = np.dtype('>u1' if message.is_bigendian else '<u1')
        point_count = len(message.data) // message.point_step
        data = np.frombuffer(
            bytearray(message.data), dtype=dtype
        ).reshape(point_count, message.point_step)

        float_dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        x_offset, y_offset, z_offset = (
            fields['x'].offset, fields['y'].offset, fields['z'].offset)
        x_gz = data[:, x_offset:x_offset + 4].view(float_dtype)[:, 0].copy()
        y_gz = data[:, y_offset:y_offset + 4].view(float_dtype)[:, 0].copy()
        z_gz = data[:, z_offset:z_offset + 4].view(float_dtype)[:, 0].copy()

        # Gazebo sensor frame (+X forward, +Y left, +Z up) to ROS optical
        # frame (+Z forward, +X right, +Y down).
        data[:, x_offset:x_offset + 4] = (-y_gz).view('u1').reshape(-1, 4)
        data[:, y_offset:y_offset + 4] = (-z_gz).view('u1').reshape(-1, 4)
        data[:, z_offset:z_offset + 4] = x_gz.view('u1').reshape(-1, 4)

        converted = PointCloud2()
        converted.header.stamp = message.header.stamp
        converted.header.frame_id = self._target_frame
        converted.height = message.height
        converted.width = message.width
        converted.fields = message.fields
        converted.is_bigendian = message.is_bigendian
        converted.point_step = message.point_step
        converted.row_step = message.row_step
        converted.data = data.reshape(-1).tobytes()
        converted.is_dense = message.is_dense
        self._publisher.publish(converted)


def main():
    rclpy.init()
    node = GazeboPointCloudToOptical()
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
