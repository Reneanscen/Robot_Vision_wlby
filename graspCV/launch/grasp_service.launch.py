from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="graspCV",
            executable="grasp_cv_node",
            output="screen",
            parameters=[{
                "max_actives": 3,
                "service_name": "grasp_cv_service",
                "realsense_topic_name": "/realsense/image_raw",
                "kinect_topic_name": "/kinect/image_raw",
            }]
        ),
    ])
