import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('inception_p0')
    
    location_file_path = os.path.join(package_dir, 'config', 'locations.yaml')
    person_pose_path = os.path.join( package_dir, 'config', 'person_pose.yaml')
    
    # Number of robots
    n_robots = 2  # Set the number of robots

    nodes = []
    for i in range(1, n_robots + 1):
        robot_name = f'tb{i}'
        namespace = f'/{robot_name}'  # Namespace for each robot

        autonomy_node_cmd = Node(
            package="inception_p0",
            executable="autonomy_node",
            name="autonomy_node",
            parameters=[{
                "location_file": location_file_path,
                "person_file": person_pose_path,
                "robot_name": robot_name 
            }],
            namespace=namespace,
            remappings=[
                ('/tf', f'{namespace}/tf'),
                ('/tf_static', f'{namespace}/tf_static'),
                ('/navigate_to_pose', f'{namespace}/navigate_to_pose'),
            ]
        )

        # Add the autonomy node to the list of nodes
        nodes.append(autonomy_node_cmd)

    # Create and return the LaunchDescription
    ld = LaunchDescription()
    for node in nodes:
        ld.add_action(node)

    return ld
