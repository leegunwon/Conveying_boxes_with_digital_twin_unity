import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # String 메시지 타입 추가
from sensor_msgs.msg import CompressedImage  # CompressedImage 메시지 타입 추가
import json
import cv2
import numpy as np
from io import BytesIO

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.ser = None
        self.data = None
        self.status = String()
        
        # 'conveyor/control' 메시지 구독
        self.subscription = self.create_subscription(
            String,  # 메시지 타입을 String으로 변경
            'conveyor/control',  # 토픽 이름
            self.listener_callback,
            10  # 큐 크기
        )

        self.publisher = self.create_publisher(String, 'conveyor/status', 10)
        
        # 변수 유지 (안 하면 가비지 컬렉션됨)
        self.status.data = "READY"
        self.publisher.publish(self.status)
        
        # 0.01초마다 시리얼 데이터 읽는 타이머 설정
        self.create_timer(0.01, self.read_data_from_arduino)

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        control = data.get("control")
        distance = str(int(float(data.get("distance.mm", "0.093")) * 10.8)) + "\n"

        # 시리얼로 거리 전송
        self.ser.write(distance.encode())  # 문자열을 바이트로 변환 후 전송
        self.get_logger().info(f"Sent: {distance.strip()}")  # 전송된 데이터 출력
        
        # 제어 메시지에 따른 상태 변경
        if control == "go":            
            self.status.data = "RUN"
            self.publisher.publish(self.status)
        elif control == "stop":
            self.status.data = "READY"
            self.publisher.publish(self.status)
        
        # 잠시 대기 (아두이노와 안정적인 통신을 위해)
        time.sleep(0.5)
        
    def serial_connect(self):
        # 시리얼 포트 설정
        port = "/dev/ttyACM0"  # 연결된 포트
        baud_rate = 115200      # 아두이노의 시리얼 통신 속도와 맞춰야 함

        # 시리얼 포트 열기
        self.ser = serial.Serial(port, baud_rate, timeout=1)

        # 아두이노와 안정적인 연결을 위해 대기
        time.sleep(2) 
        
        print("Serial communication started.")
        self.status.data = "INIT"
        self.publisher.publish(self.status)  # 초기화 메시지 전송

    def read_data_from_arduino(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port not open")

        # 시리얼 포트에서 데이터 읽기
        if self.ser.in_waiting > 0:
            data = self.ser.read(1).decode().strip()

            if self.data != data:
                if data == '.':
                    self.status.data = "READY"
                    self.publisher.publish(self.status)
                    self.data = data
                elif data == '_':
                    self.status.data = "RUN"
                    self.publisher.publish(self.status)
                    self.data = data
                else:
                    self.get_logger().warn(f"Unknown command received: {data}")

    def serial_connect(self):
        """시리얼 포트 연결 시도"""
        # 시리얼 포트 설정
        port = "/dev/ttyACM0"  # 연결된 포트
        baud_rate = 115200      # 아두이노의 시리얼 통신 속도와 맞춰야 함

        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(2)  # 아두이노와 안정적인 연결을 위해 대기
                self.get_logger().info("Serial communication started.")
                self.status.data = "READY"
                self.publisher.publish(self.status)  # 초기화 메시지 전송
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to serial port: {e}")
                self.status.data = "DISCONNECT"
                self.publisher.publish(self.status)  # 연결 실패 상태 전송
                return

    def check_serial_connection(self):
        """시리얼 연결 상태 체크"""
        if self.ser and self.ser.is_open:
            return True
        return False

    def check_connection_periodically(self):
        """시리얼 포트 상태를 주기적으로 확인"""
        while rclpy.ok():
            if not self.check_serial_connection():
                self.get_logger().warn("Serial connection lost. Attempting to reconnect...")
            else:
                self.get_logger().info("Serial connection is active.")
            time.sleep(5)  # 5초마다 연결 상태 확인

def main():
    rclpy.init()
    node = MySubscriber()
    try:
        node.serial_connect()  # 시리얼 통신 시작
        rclpy.spin(node)  # 구독을 지속적으로 대기
    except KeyboardInterrupt:
        print("Serial communication stopped.")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.ser.close()  # 시리얼 포트 닫기

if __name__ == '__main__':
    main()



def main():
    rclpy.init()
    node = MySubscriber()
    try:
        node.serial_connect()  # 시리얼 통신 시작
        rclpy.spin(node)  # 구독을 지속적으로 대기
    except KeyboardInterrupt:
        print("Serial communication stopped.")
        pass
    finally:
        if node.ser:
            node.ser.close()  # 시리얼 포트 닫기
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
