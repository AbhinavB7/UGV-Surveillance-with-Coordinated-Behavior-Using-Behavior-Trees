#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct
from scipy.spatial import KDTree

class RedCubeObstaclePublisher(Node):
    def __init__(self):
        super().__init__('red_cube_obstacle_publisher')

        # Get the namespace from the node's fully qualified name
        self.namespace = self.get_namespace()

        # Initialize subscribers to RGB, depth, and camera info topics
        self.rgb_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Initialize the publisher for the obstacles point cloud
        self.obstacle_publisher = self.create_publisher(PointCloud2, f'{self.namespace}/obstacles', 10)

        # create a timer to publish the obstacles point cloud
        self.create_timer(0.05, self.publish_combined_obstacles)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.radius = 0.4  # Radius of the obstacle sphere
        self.num_points_per_obstacle = 2000  # Number of points per obstacle
        self.proximity_threshold = 1.0  # Threshold distance in meters

        # List to store unique 3D points in the map frame
        self.obstacle_points = []

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def rgb_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_red_cube()

    def depth_callback(self, msg):
        # Convert ROS depth image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detect_red_cube(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return

        # Convert the RGB image to HSV color space
        hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        # Define the range for detecting red color
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask for red color
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours of the red areas
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming it is the red cube)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw the bounding box on the RGB image
            cv2.rectangle(self.rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Calculate the center of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2

            # Get the distance (depth) from the depth image at the center of the bounding box
            distance = self.depth_image[center_y, center_x]

            # Get camera intrinsic parameters
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # Calculate the 3D coordinates of the cube in the camera frame
            x_3d = (center_x - cx) * distance / fx
            y_3d = (center_y - cy) * distance / fy
            z_3d = distance

            # Create a PointStamped message for the 3D point in the camera frame
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header.frame_id = 'camera_rgb_optical_frame'
            point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
            point_in_camera_frame.point.x = x_3d
            point_in_camera_frame.point.y = y_3d
            point_in_camera_frame.point.z = z_3d/1.0

            # Listen to the transform between map and camera_rgb_optical_frame
            try:
                transform = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time())

                # Transform the point to the map frame
                point_in_map_frame = do_transform_point(point_in_camera_frame, transform)

                x_map = point_in_map_frame.point.x
                y_map = point_in_map_frame.point.y
                z_map = point_in_map_frame.point.z

                print(f"[{self.namespace}] 3D Point in Map Frame: {x_map:.2f}, {y_map:.2f}, {z_map:.2f}")

                # Check if the point is within 1 meter of any existing points
                if not self.is_within_radius((x_map, y_map, z_map), self.proximity_threshold):
                    # If the point is unique, add it to the list
                    if z_map < 0.22:  # Ignore points too close to camera
                        self.obstacle_points.append((x_map, y_map, z_map))

                    cv2.putText(self.rgb_image, f"Map Frame X: {x_map:.2f} Y: {y_map:.2f} Z: {z_map:.2f} meters", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            except TransformException as e:
                self.get_logger().warn(f"[{self.namespace}] Could not transform from camera_rgb_optical_frame to map: {str(e)}")

      

        cv2.imshow(f'Red Cube Detection [{self.namespace}]', self.rgb_image)
        cv2.waitKey(1)

    def is_within_radius(self, new_point, radius):
        """Check if the new point is within a specified radius of any existing points."""
        if len(self.obstacle_points) == 0:
            return False

        # Create a KDTree for efficient radius search
        tree = KDTree(self.obstacle_points)
        indices = tree.query_ball_point(new_point, radius)

        return len(indices) > 0

    def generate_sphere_points(self, center, radius, num_points):
        """Generate points inside a sphere with a given radius and center."""
        points = []
        for _ in range(num_points):
            # Random spherical coordinates
            theta = 2 * np.pi * np.random.random()  # Azimuthal angle
            phi = np.arccos(2 * np.random.random() - 1)  # Polar angle
            r = radius * np.cbrt(np.random.random())  # Radial distance

            # Convert spherical coordinates to Cartesian
            x = r * np.sin(phi) * np.cos(theta) + center[0]
            y = r * np.sin(phi) * np.sin(theta) + center[1]
            z = r * np.cos(phi) + center[2]

            points.append((x, y, z))
        return np.array(points, dtype=np.float32)

    def publish_combined_obstacles(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Generate points for all stored obstacle locations
        all_points = []
        for point in self.obstacle_points:
            sphere_points = self.generate_sphere_points(point, self.radius, self.num_points_per_obstacle)
            all_points.extend(sphere_points)

        # Create the PointCloud2 message
        point_cloud_data = []
        for point in all_points:
            point_cloud_data.append(struct.pack('fff', *point))

        # Flatten the list of packed points
        point_cloud_data = b''.join(point_cloud_data)

        point_cloud = PointCloud2()
        point_cloud.header = header
        point_cloud.height = 1  # Single layer of points
        point_cloud.width = len(all_points)  # Number of points
        point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12  # 3 floats * 4 bytes
        point_cloud.row_step = point_cloud.point_step * point_cloud.width
        point_cloud.data = point_cloud_data
        point_cloud.is_dense = True

        self.obstacle_publisher.publish(point_cloud)

def main(args=None):
    rclpy.init(args=args)

    red_cube_obstacle_publisher = RedCubeObstaclePublisher()

    rclpy.spin(red_cube_obstacle_publisher)
    red_cube_obstacle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
