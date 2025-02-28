import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_manipulation_bringup', 'hardware.launch.py'],
            output='screen',
        ),
    ]

    return launch.LaunchDescription(nodes)
