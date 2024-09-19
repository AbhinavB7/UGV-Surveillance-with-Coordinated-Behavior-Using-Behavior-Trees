#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Number of robots
    n_robots = 2  # Set the number of robots

    for i in range(1, n_robots + 1):
        robot_name = f'tb{i}'
        namespace = f'/{robot_name}'  # Namespace for each robot

        # Node to detect the red cube for each robot
        red_cube_detector_node = Node(
            package='inception_p0', 
            executable='red_cube_detect.py',
            name=f'{robot_name}_red_cube_detector',
            output='screen',
            namespace=namespace,
            remappings=[
                ('/camera/image_raw', f'{namespace}/camera/image_raw'),
                ('/camera/depth/image_raw', f'{namespace}/camera/depth/image_raw'),
                ('/camera/camera_info', f'{namespace}/camera/camera_info'),
                ('/tf', f'{namespace}/tf'),
                ('/tf_static', f'{namespace}/tf_static')
            ]
        )
        
        human_detector_node = Node(
            package='inception_p0',
            executable='yolo_v2.py',
            name=f'{robot_name}_yolo_detector',
            output='screen',
            namespace=namespace,
            remappings=[
                ('/camera/image_raw', f'{namespace}/camera/image_raw'),
                ('/camera/depth/image_raw', f'{namespace}/camera/depth/image_raw'),
                ('/camera/camera_info', f'{namespace}/camera/camera_info'),
                ('/tf', f'{namespace}/tf'),
                ('/tf_static', f'{namespace}/tf_static')
            ]
        )
        
        combine_obstacle_node = Node(
            package='inception_p0',
            executable='combine_obs.py',
            output='screen',
            namespace=namespace,
        )

        # Add the node to the launch description
        ld.add_action(red_cube_detector_node)
        ld.add_action(human_detector_node)
        ld.add_action(combine_obstacle_node)
                
    return ld
