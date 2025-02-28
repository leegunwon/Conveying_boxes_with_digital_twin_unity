#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from geometry_msgs.msg import Pose, PoseArray, Twist
from srv_call_test import TurtlebotArmClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import getkey
from std_msgs.msg import Header
import math
from sensor_msgs.msg import JointState
import os

# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화

j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150


th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset

# author : karl.kwon (mrthinks@gmail.com)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J0 to J2
def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)

    return s1, s2

# author : karl.kwon (mrthinks@gmail.com)
# x, y, z : relational position from J0 (joint 0)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J2 to J3
# sr1 : angle between z-axis to J0->J1
# sr2 : angle between J0->J1 to J1->J2
# sr3 : angle between J1->J2 to J2->J3 (maybe always parallel)
def solv_robot_arm2(x, y, z, r1, r2, r3):
    z = z + r3 - j1_z_offset

    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    #   Sxy = math.acos(x / Rxy)
    Sxy = math.atan2(y, x)

    s1, s2 = solv2(r1, r2, Rt)

    sr1 = math.pi/2 - (s1 + St)
    sr2 = s1 + s2
    sr2_ = sr1 + sr2
    sr3 = math.pi - sr2_

    return Sxy, sr1, sr2, sr3, St, Rt

##YoloDetect
class YoloDetect(Node):
    def __init__(self):

        super().__init__('yolo_detect')        
        self.subscription = self.create_subscription(
            String,
            '/yolo/detected_info',
            self.listener_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        self.twist = Twist()


        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()

        Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(100, 0, 0, r1, r2, r3)


        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        #self.trajectory_msg.header.stamp = current_time.to_msg()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        self.trajectory_msg.points = [point]
        self.joint_pub.publish(self.trajectory_msg)
  
        self.yolofind = False
        self.armrun = False
        self.yolo_x = 0
        self.yolo_y = 0

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_joint = False


    def joint_states_callback(self, msg):
        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def listener_callback(self, msg):
        if not self.armrun:  # 로봇 암이 동작 중이 아니면
            self.data = msg.data
            try:
                data_list = ast.literal_eval(self.data)
                print(data_list)

                # 데이터가 예상한 구조인지 확인
                if len(data_list) > 0 :
                    self.yolo_x = data_list[0][1]
                    self.yolo_y = data_list[0][2]
                    
                    print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                    # print("done")

                    # if not self.yolofind:
                    #     self.yolofind = True
                    #     self.arm_controll()
                else:
                    self.get_logger().warn("Invalid data format: data_list is empty or does not have the expected structure.")
            
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")


    def append_pose_init(self, x, y, z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose_array.poses.append(pose)

        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array
        
    def arm_controll(self):
        arm_client = TurtlebotArmClient()
        # 서비스 요청을 해서 moveit에게 움직이라고 요청함
        print ("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.055, 0.0 - self.yolo_x ,0.122354 )

            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
            
            time.sleep(1)
            # 0은 주어진 좌표 이동 

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.055, 0.0 - self.yolo_x ,0.087354  )

            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
   
            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print ("conveyor task start")

            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(3)

            print("jobs_done")


            self.armrun = False
            self.yolofind = False  # 작업 완료 후 초기화

joint_angle_delta = 0.05  # radian

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    #rclpy.spin(node)

    arm_client = TurtlebotArmClient()
    #print ("task start!")

    lx = 0.14
    ly = 0.0
    lz = 0.23
    '''
    right_low_x_offset =  0.005
    right_low_y_offset = -0.005

    right_high_x_offset = -0.01
    right_high_y_offset = -0.005

    left_low_x_offset = 0.00
    left_low_y_offset = 0.005

    left_high_x_offset = -0.005
    left_high_y_offset = 0.005
    '''


    right_low_x_offset = 0.0
    right_low_y_offset = 0.0

    right_high_x_offset = 0.0
    right_high_y_offset = 0.0

    left_low_x_offset = 0.0
    left_low_y_offset = 0.0

    left_high_x_offset = 0.0
    left_high_y_offset = 0.0

    # 파일 경로 설정
    file_path = 'offset_values.txt'

    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            for line in file:
                # 각 줄을 "변수명 : 값" 형식으로 분리하여 변수에 값을 넣음
                parts = line.strip().split(":")
                if len(parts) == 2:
                    var_name, value = parts
                    try:
                        # 값을 float로 변환하여 변수에 할당
                        value = float(value.strip())
                        if var_name == "right_low_x_offset ":
                            right_low_x_offset = value
                        elif var_name == "right_low_y_offset ":
                            right_low_y_offset = value
                        elif var_name == "right_high_x_offset ":
                            right_high_x_offset = value
                        elif var_name == "right_high_y_offset ":
                            right_high_y_offset = value
                        elif var_name == "left_low_x_offset ":
                            left_low_x_offset = value
                        elif var_name == "left_low_y_offset ":
                            left_low_y_offset = value
                        elif var_name == "left_high_x_offset ":
                            left_high_x_offset = value
                        elif var_name == "left_high_y_offset ":
                            left_high_y_offset = value
                    except ValueError:
                        pass
    else:
        # 파일이 없으면 새로운 파일을 생성하고 값을 저장
        with open(file_path, "w") as file:
            file.write(f"right_low_x_offset : {right_low_x_offset}\n")
            file.write(f"right_low_y_offset : {right_low_y_offset}\n")
            file.write(f"right_high_x_offset : {right_high_x_offset}\n")
            file.write(f"right_high_y_offset : {right_high_y_offset}\n")
            file.write(f"left_low_x_offset : {left_low_x_offset}\n")
            file.write(f"left_low_y_offset : {left_low_y_offset}\n")
            file.write(f"left_high_x_offset : {left_high_x_offset}\n")
            file.write(f"left_high_y_offset : {left_high_y_offset}\n")

    yolo_robot_x = 0.0
    yolo_robot_y = 0.0

    response = arm_client.send_request(1, "camera_home")
    arm_client.get_logger().info(f'Response: {response.response}')
    time.sleep(1)

    while(rclpy.ok()):
        key_value = getkey.getkey()
        print(f"No. {key_value}")
        if key_value == '1':
            rclpy.spin_once(node)
            node.armrun = False
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')        
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            time.sleep(1)

        elif key_value == '2':
            rclpy.spin_once(node)
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '3':
            # rclpy.spin_once(node)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)

            if node.yolo_x > 0 and node.yolo_y > 0:   # right low
                yolo_robot_y = node.yolo_x + right_low_y_offset       # hight  += 아래   -= 위
                yolo_robot_x = node.yolo_y + right_low_x_offset       # width  += 오른쪽 -= 왼쪽
                print(f'right low  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_x}]')
                # x : 0.03522528820800782] y : 0.0352 2528820800782]
            elif node.yolo_x > 0 and node.yolo_y < 0:  # right high
                yolo_robot_y = node.yolo_x + right_high_y_offset       # hight  += 아래   -= 위
                yolo_robot_x = node.yolo_y + right_high_x_offset       # width  += 오른쪽 -= 왼쪽
                #print("right high")
                print(f'right high  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_x}]')
            elif node.yolo_x < 0 and node.yolo_y > 0:  # left low
                yolo_robot_y = node.yolo_x + left_low_y_offset       # hight  += 아래   -= 위
                yolo_robot_x = node.yolo_y + left_low_x_offset       # width  += 오른쪽 -= 왼쪽
                #print("left low")
                print(f'left low  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_x}]')
            elif node.yolo_x < 0 and node.yolo_y < 0:  # left high
                yolo_robot_y = node.yolo_x + left_high_y_offset       # hight  += 아래   -= 위
                yolo_robot_x = node.yolo_y + left_high_x_offset       # width  += 오른쪽 -= 왼쪽
                #print("left high")
                print(f'left high  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_x}]')

            print(yolo_robot_x, yolo_robot_y)
            print(yolo_robot_x, yolo_robot_y)
            print(yolo_robot_x, yolo_robot_y)
            print(yolo_robot_x, yolo_robot_y)

            #pose_array = node.append_pose_init(0.137496 - node.yolo_y + 0.055, 0.0 - node.yolo_x ,0.122354 )
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2, 0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '4':
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2, 0.087354  )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == '5':
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '6':
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '7':
            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '8':
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '9':
            print(f'right low  :  yolo_robot_x [{right_low_x_offset}] yolo_robot_y[{right_low_y_offset}]')
            print(f'right high :  yolo_robot_x [{right_high_x_offset}] yolo_robot_y[{right_high_y_offset}]')
            print(f'left low   :  yolo_robot_x [{left_low_x_offset}] yolo_robot_y[{left_low_y_offset}]')
            print(f'left high  :  yolo_robot_x [{left_high_x_offset}] yolo_robot_y[{left_high_y_offset}]')

        elif key_value == '0':
            # 값을 파일에 저장
            with open(file_path, "w") as file:
                file.write(f"right_low_x_offset : {right_low_x_offset}\n")
                file.write(f"right_low_y_offset : {right_low_y_offset}\n")
                file.write(f"right_high_x_offset : {right_high_x_offset}\n")
                file.write(f"right_high_y_offset : {right_high_y_offset}\n")
                file.write(f"left_low_x_offset : {left_low_x_offset}\n")
                file.write(f"left_low_y_offset : {left_low_y_offset}\n")
                file.write(f"left_high_x_offset : {left_high_x_offset}\n")
                file.write(f"left_high_y_offset : {left_high_y_offset}\n")
            print("Values saved.")

        elif key_value == 'a':
            pass
            
        elif key_value == 'q':
            break

        #######################  right low   #######################
        # right low  :  yolo_robot_x [0.005] yolo_robot_y[-0.005]
        elif key_value == 'w':
            right_low_x_offset -= 0.005              # hight  += 아래   -= 위
            yolo_robot_x -= 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 's':
            right_low_x_offset += 0.005              # hight  += 아래   -= 위
            yolo_robot_x += 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'e':
            right_low_y_offset += 0.005             # width  += 오른쪽 -= 왼쪽
            yolo_robot_y += 0.005                   # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'd':
            right_low_y_offset -= 0.005             # width  += 오른쪽 -= 왼쪽
            yolo_robot_y -= 0.005                   # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        #######################  right high   #######################
        # right low  :  yolo_robot_x [-0.035] yolo_robot_y[-0.01]
        elif key_value == 'r':
            right_high_x_offset -= 0.005             # hight  += 아래   -= 위
            yolo_robot_x -= 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'f':
            right_high_x_offset += 0.005             # hight  += 아래   -= 위
            yolo_robot_x += 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 't':
            right_high_y_offset += 0.005             # width  += 오른쪽 -= 왼쪽
            yolo_robot_y += 0.005                    # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'g':
            right_high_y_offset -= 0.005              # width  += 오른쪽 -= 왼쪽
            yolo_robot_y -= 0.005                    # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        #######################  left low   #######################
        # right high :  yolo_robot_x [0.01] yolo_robot_y[0.01]
        elif key_value == 'y':
            left_low_x_offset -= 0.005              # hight  += 아래   -= 위
            yolo_robot_x -= 0.005                   # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'h':
            left_low_x_offset += 0.005              # hight  += 아래   -= 위
            yolo_robot_x += 0.005                   # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'u':
            left_low_y_offset += 0.005              # width  += 오른쪽 -= 왼쪽
            yolo_robot_y += 0.005                   # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'j':
            left_low_y_offset -= 0.005              # width  += 오른쪽 -= 왼쪽
            yolo_robot_y -= 0.005                   # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        #######################  left high   #######################
        elif key_value == 'i':
            left_high_x_offset -= 0.005              # hight  += 아래   -= 위
            yolo_robot_x -= 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'k':
            left_high_x_offset += 0.005              # hight  += 아래   -= 위
            yolo_robot_x += 0.005                    # hight  += 아래   -= 위
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'o':
            left_high_y_offset += 0.005              # width  += 오른쪽 -= 왼쪽
            yolo_robot_y += 0.005                    # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == 'l':
            left_high_y_offset -= 0.005              # width  += 오른쪽 -= 왼쪽
            yolo_robot_y -= 0.005                    # width  += 오른쪽 -= 왼쪽
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2 ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        ############################################################
        elif key_value == '>':                       # forward
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '<':                       # backward
            print(f"No. {key_value}")
            node.twist.linear.x = -0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '?':                       # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
            node.get_joint = False

        elif key_value == 'A':
            print(f"No. {key_value}")
            node.arm_controll()

    print ("task end!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()