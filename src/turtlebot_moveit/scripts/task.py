#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
import time

def main(args=None):
    rclpy.init(args=args)
    arm_client = TurtlebotArmClient()

    print ("task start!")


    # ## gripper control
    # response = arm_client.send_request(2, "close")
    # arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(9, "")
    arm_client.get_logger().info(f'Response: {response.response}')
    

    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()