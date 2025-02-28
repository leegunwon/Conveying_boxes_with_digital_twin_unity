#!/usr/bin/env python3
## ros2 run turtlebot_moveit srv_call_test.py 2 close
## cmd =0  waypoint 이동
## cmd =1  arm named target 이동
## cmd =2  gripper named target 이동
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
import argparse

class TurtlebotArmClient(Node):

    def __init__(self):
        super().__init__('turtlebot_arm_client')
        self.client = self.create_client(MoveitControl, 'moveit_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveitControl.Request()

    def send_request(self, cmd, posename='', waypoints=None):
        self.req.cmd = cmd
        self.req.posename = posename
        if waypoints:
            self.req.waypoints = waypoints
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Send MoveitControl command to TurtlebotArmClient')
    parser.add_argument('cmd', type=int, help='Command to send (e.g., 0 for waypoints, 1 for named target)')
    parser.add_argument('posename', type=str, help='Pose name for the command')
    parsed_args = parser.parse_args()

    client = TurtlebotArmClient()

    # Example usage
    response = client.send_request(parsed_args.cmd, parsed_args.posename)
    client.get_logger().info(f'Response: {response.response}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()