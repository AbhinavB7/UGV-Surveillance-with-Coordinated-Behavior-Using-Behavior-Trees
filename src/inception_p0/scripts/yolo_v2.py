#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import torch
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.namespace = self.get_namespace()
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.listener_callback, 
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image, 
            '/camera/depth/image_raw', 
            self.depth_callback, 
            10
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.get_logger().info("YOLOv5 model loaded successfully")
        self.depth_image = None
        self.camera_info = None
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the person's position
        self.person_pub = self.create_publisher(PoseStamped, '/person', 10)

    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Perform inference with YOLOv5
        results = self.model(cv_image)

        # Filter for human detections with confidence above 0.75
        for detection in results.xyxy[0]: 
            if int(detection[5]) == 0 and detection[4] > 0.75:  # class 0 corresponds to 'person'
                x1, y1, x2, y2 = map(int, detection[:4])
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                self.get_logger().info(f'Human detected with confidence {detection[4]} at center ({center_x}, {center_y})')

                # Calculate the human's position using the depth camera and camera parameters
                human_position = None
                if self.depth_image is not None and self.camera_info is not None:
                    human_position = self.calculate_human_position(center_x, center_y)
                    if human_position is not None:
                        self.get_logger().info(f'Human position (map frame): X={human_position[0]:.2f}, Y={human_position[1]:.2f}, Z={human_position[2]:.2f}')
                        self.publish_person_position(human_position)

                        # Annotate the frame
                        cv2.putText(cv_image, f'Person {detection[4]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # Annotate the frame with the bounding box regardless of human position
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the resulting frame in a single window
        cv2.imshow(f'YOLO Human Detection [{self.namespace}]', cv_image)

        # Close the window when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def depth_callback(self, data):
        # Convert ROS depth image to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

    def camera_info_callback(self, info):
        # Store the camera intrinsic parameters
        self.camera_info = info

    def calculate_human_position(self, center_x, center_y):
        x_map = y_map = z_map = None

        #if self.depth_image is not None and center_y < self.depth_image.shape[0] and center_x < self.depth_image.shape[1]:
        depth_value = self.depth_image[center_y, center_x]

        if depth_value > 0:
            fx = self.camera_info.k[0]  # Focal length x
            fy = self.camera_info.k[4]  # Focal length y
            cx = self.camera_info.k[2]  # Principal point x
            cy = self.camera_info.k[5]  # Principal point y

            # Calculate the 3D point in the camera frame
            x = (center_x - cx) * depth_value / fx 
            y = (center_y - cy) * depth_value / fy
            z = depth_value 

            self.get_logger().info(f"Camera frame position: X={x}, Y={y}, Z={z}")

            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header.frame_id = 'camera_rgb_optical_frame'
            point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
            point_in_camera_frame.point.x = x
            point_in_camera_frame.point.y = y
            point_in_camera_frame.point.z = z/1.0
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'camera_rgb_optical_frame',
                    rclpy.time.Time())

                point_in_map_frame = do_transform_point(point_in_camera_frame, transform)

                x_map = point_in_map_frame.point.x
                y_map = point_in_map_frame.point.y
                z_map = point_in_map_frame.point.z

            except TransformException as e:
                self.get_logger().warn(f"[{self.namespace}] Could not transform from camera_rgb_optical_frame to map: {str(e)}")

        if x_map is not None and y_map is not None and z_map is not None:
            return (x_map, y_map, z_map)
        else:
            return None


    def publish_person_position(self, human_position):
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Set the position
        pose_msg.pose.position.x = human_position[0]
        pose_msg.pose.position.y = human_position[1]
        pose_msg.pose.position.z = human_position[2]

        # Orientation can be set to a default since it's not being computed here
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # Publish the pose to the /person topic
        self.person_pub.publish(pose_msg)
        self.get_logger().info(f'Published human position to /person topic: X={human_position[0]:.2f}, Y={human_position[1]:.2f}, Z={human_position[2]:.2f}')


def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
