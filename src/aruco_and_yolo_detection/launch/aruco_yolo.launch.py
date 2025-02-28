import launch
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package='aruco_and_yolo_detection',
            executable='compressed_image_publihser',
            name='compressed_image_publihser',
            output='screen', 
        ),
        
        Node(
            package='aruco_and_yolo_detection',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',  
        ),
                
        Node(
            package='aruco_and_yolo_detection',
            executable='yolo_detect',
            name='yolo_detect',
            output='screen',
        ),
    ]

    return launch.LaunchDescription(nodes)
