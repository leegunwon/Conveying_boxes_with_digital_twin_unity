import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # 기본 실행할 노드들
    nodes = [
        Node(
            package='conveyor_system_gui',
            executable='gui',
            name='gui',
            output='screen',    
        ),
        
        Node(
            package='conveyor_controller',
            executable='conveyor_node',
            name='conveyor_node',
            output='screen',  
        ),
                
        Node(
            package='turtlebot_moveit',
            executable='turtlebot_arm_controller',
            output='screen',
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_manipulation_moveit_config', 'moveit_core.launch.py'],
            output='screen',
        ),

        # 실행할 노드를 결정하는 런치 인자 추가
        DeclareLaunchArgument(
            'execute_node',
            default_value='',
            description='Node executable to launch'
        ),
    ]

    return launch.LaunchDescription(nodes)
