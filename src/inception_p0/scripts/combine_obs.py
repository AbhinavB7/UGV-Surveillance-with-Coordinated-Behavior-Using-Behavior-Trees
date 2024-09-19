#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class CombinedObstaclePublisher(Node):
    def __init__(self):
        super().__init__('combined_obstacle_publisher')

        # Subscribers for the obstacle point clouds from tb1 and tb2
        self.tb1_subscriber = self.create_subscription(
            PointCloud2,
            '/tb1/obstacles',
            self.tb1_callback,
            10
        )

        self.tb2_subscriber = self.create_subscription(
            PointCloud2,
            '/tb2/obstacles',
            self.tb2_callback,
            10
        )

        # Publisher for the combined obstacle point cloud
        self.combined_obstacle_publisher = self.create_publisher(PointCloud2, '/obstacles', 10)

        # Variables to store the latest point cloud data
        self.tb1_point_cloud = None
        self.tb2_point_cloud = None

    def tb1_callback(self, msg):
        self.tb1_point_cloud = msg
        self.combine_and_publish()

    def tb2_callback(self, msg):
        self.tb2_point_cloud = msg
        self.combine_and_publish()

    def combine_and_publish(self):
        if self.tb1_point_cloud is None or self.tb2_point_cloud is None:
            return

        # Combine the two point clouds
        combined_points = self.combine_point_clouds(self.tb1_point_cloud, self.tb2_point_cloud)

        # Create a new PointCloud2 message for the combined point cloud
        combined_point_cloud = PointCloud2()
        combined_point_cloud.header = Header()
        combined_point_cloud.header.stamp = self.get_clock().now().to_msg()
        combined_point_cloud.header.frame_id = 'map'
        combined_point_cloud.height = 1
        combined_point_cloud.width = len(combined_points)
        combined_point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        combined_point_cloud.is_bigendian = False
        combined_point_cloud.point_step = 12  # 3 floats * 4 bytes
        combined_point_cloud.row_step = combined_point_cloud.point_step * combined_point_cloud.width
        combined_point_cloud.is_dense = True

        # Pack the combined points into the point cloud data
        combined_point_cloud.data = b''.join([struct.pack('fff', *point) for point in combined_points])

        # Publish the combined point cloud
        self.combined_obstacle_publisher.publish(combined_point_cloud)
        self.get_logger().info(f'Published combined point cloud with {combined_point_cloud.width} points')

    def combine_point_clouds(self, cloud1, cloud2):
        # Unpack the point data from both clouds
        points1 = self.unpack_point_cloud(cloud1)
        points2 = self.unpack_point_cloud(cloud2)

        # Combine the points
        combined_points = points1 + points2
        return combined_points

    def unpack_point_cloud(self, cloud):
        points = []
        for i in range(cloud.width):
            offset = i * cloud.point_step
            x, y, z = struct.unpack_from('fff', cloud.data, offset)
            points.append((x, y, z))
        return points

def main(args=None):
    rclpy.init(args=args)
    node = CombinedObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
