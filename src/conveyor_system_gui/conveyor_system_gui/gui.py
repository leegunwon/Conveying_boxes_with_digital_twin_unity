import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox, QHBoxLayout, QTextEdit
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtGui import QImage, QPixmap, QFont
import json
import numpy as np
from cv_bridge import CvBridge
import cv2
import os

# Qt 환경 변수 설정
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/qt5/plugins'
os.environ['QT_QPA_PLATFORM'] = 'xcb'


class RosPublisher(Node):
    def __init__(self, ui_callback, image_callback, log_callback):
        super().__init__('ros_publisher')
        self.publisher_conveyor = self.create_publisher(String, 'conveyor/control', 10)

        self.publisher = self.create_publisher(String, 'gui/command', 10)

        self.subscription = self.create_subscription(
            String, 'conveyor/status', self.status_callback, 10
        )  

        self.image_subscription = self.create_subscription(
            CompressedImage, 'yolo/compressed', self.image_callback, 10
        )


        self.ui_callback = ui_callback
        self.image_callback = image_callback
        self.bridge = CvBridge()
        self.log_callback = log_callback

    def status_callback(self, msg):
        status = msg.data
        print(f"📢 Conveyor 상태 수신: {status}")
        self.ui_callback.emit(status)
        self.log_callback.emit(f"📢 Conveyor 상태 수신: {status}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.image_callback.emit(cv_image)
        except Exception as e:
            print(f"Error converting compressed image: {e}")
            self.log_callback.emit(f"Error converting compressed image: {e}")



    def publish_message(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {data}')
        self.log_callback.emit(f'Published: {data}')


    def publish_conv_message(self, data):
        msg = String()
        msg.data = data
        self.publisher_conveyor.publish(msg)
        self.get_logger().info(f'Published: {data}')
        self.log_callback.emit(f'Published: {data}')


class RosThread(QThread):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

    def run(self):
        rclpy.spin(self.ros_node)


class MyApp(QWidget):
    status_signal = pyqtSignal(str)
    image_signal = pyqtSignal(object)
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.init_ui()

        # ROS2 노드 초기화
        rclpy.init()
        self.ros_node = RosPublisher(self.status_signal, self.image_signal, self.log_signal)

        self.status_signal.connect(self.update_status)
        self.image_signal.connect(self.update_image)
        self.log_signal.connect(self.append_to_log)

        # ROS2 스레드 실행
        self.ros_thread = RosThread(self.ros_node)
        self.ros_thread.start()

    def init_ui(self):
        self.setWindowTitle('PyQt with ROS2 Publisher')
        self.setStyleSheet("background-color: #f4f4f9;")  # 배경 색상 변경

        main_layout = QHBoxLayout()  # 메인 레이아웃을 QHBoxLayout으로 변경

        # 왼쪽 레이아웃 (기존 요소들)
        left_layout = QVBoxLayout()

        # 상태 표시 라벨
        self.status_label = QLabel('현재 상태: 대기 중', self)
        self.status_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.status_label.setStyleSheet("color: #2E8B57;")  # 녹색 텍스트
        left_layout.addWidget(self.status_label)
        # 이미지 표시용 QLabel
        self.image_label = QLabel('이미지가 없습니다.', self)
        self.image_label.setFixedSize(640, 360)
        self.image_label.setStyleSheet("background-color: lightgray; border: 1px solid #ccc;")
        left_layout.addWidget(self.image_label)

        # 작업 목록 (Job1, Job2, Job3 선택)
        self.job_label = QLabel('작업 선택:', self)
        self.job_label.setFont(QFont('Arial', 12))
        left_layout.addWidget(self.job_label)

        self.job_combo = QComboBox(self)
        self.job_combo.addItems(["Job1", "Job2", "Job3"])
        self.job_combo.setStyleSheet("background-color: #ffffff; border: 1px solid #ccc; padding: 5px; font-size: 12px;")
        left_layout.addWidget(self.job_combo)

        # 파란색 박스 수량 표시 (수평 배치)
        self.blue_box_label = QLabel('파란색 박스 수량:', self)
        self.blue_box_label.setFont(QFont('Arial', 12))
        
        # 파란색 박스 수량과 값 수평 배치
        blue_box_layout = QHBoxLayout()
        blue_box_layout.addWidget(self.blue_box_label)

        self.blue_box_count = 0
        self.blue_box_count_label = QLabel(str(self.blue_box_count), self)
        self.blue_box_count_label.setFont(QFont('Arial', 12, QFont.Bold))
        blue_box_layout.addWidget(self.blue_box_count_label)

        # 파란색 박스 수량 증가/감소 버튼 (수직 배열)
        blue_box_button_layout = QVBoxLayout()  # QVBoxLayout으로 변경
        self.blue_box_increase_button = QPushButton('▲', self)
        self.blue_box_increase_button.clicked.connect(self.increase_blue_box_count)
        self.blue_box_decrease_button = QPushButton('▼', self)
        self.blue_box_decrease_button.clicked.connect(self.decrease_blue_box_count)

        blue_box_button_layout.addWidget(self.blue_box_increase_button)
        blue_box_button_layout.addWidget(self.blue_box_decrease_button)
        blue_box_layout.addLayout(blue_box_button_layout)

        # 빨간색 박스 수량 표시 (수평 배치)
        self.red_box_label = QLabel('빨간색 박스 수량:', self)
        self.red_box_label.setFont(QFont('Arial', 12))

        # 빨간색 박스 수량과 값 수평 배치
        red_box_layout = QHBoxLayout()
        red_box_layout.addWidget(self.red_box_label)

        self.red_box_count = 0
        self.red_box_count_label = QLabel(str(self.red_box_count), self)
        self.red_box_count_label.setFont(QFont('Arial', 12, QFont.Bold))
        red_box_layout.addWidget(self.red_box_count_label)

        # 빨간색 박스 수량 증가/감소 버튼 (수직 배열)
        red_box_button_layout = QVBoxLayout()  # QVBoxLayout으로 변경
        self.red_box_increase_button = QPushButton('▲', self)
        self.red_box_increase_button.clicked.connect(self.increase_red_box_count)
        self.red_box_decrease_button = QPushButton('▼', self)
        self.red_box_decrease_button.clicked.connect(self.decrease_red_box_count)

        red_box_button_layout.addWidget(self.red_box_increase_button)
        red_box_button_layout.addWidget(self.red_box_decrease_button)
        red_box_layout.addLayout(red_box_button_layout)

        # 파란색과 빨간색 박스를 좌우로 배치
        box_layout = QHBoxLayout()  # QHBoxLayout으로 변경
        box_layout.addLayout(blue_box_layout)
        box_layout.addLayout(red_box_layout)

        left_layout.addLayout(box_layout)

        # 버튼 레이아웃
        button_layout = QHBoxLayout()
        
        # 시작 버튼
        self.start_button = QPushButton('Start', self)
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 5px; font-size: 14px; padding: 10px;")
        self.start_button.clicked.connect(self.start_publishing)
        button_layout.addWidget(self.start_button)

        # 정지 버튼
        self.conv_button = QPushButton('conveyer', self)
        self.conv_button.setStyleSheet("background-color: #f44336; color: white; border-radius: 5px; font-size: 14px; padding: 10px;")
        self.conv_button.clicked.connect(self.conv_publishing)
        button_layout.addWidget(self.conv_button)

        self.conv_stop_button = QPushButton('conv_stop', self)
        self.conv_stop_button.setStyleSheet("background-color: #f44336; color: white; border-radius: 5px; font-size: 14px; padding: 10px;")
        self.conv_stop_button.clicked.connect(self.conv_stop_publishing)
        button_layout.addWidget(self.conv_stop_button)




        left_layout.addLayout(button_layout)
        main_layout.addLayout(left_layout)

        # 오른쪽 레이아웃 (로그 출력 영역)
        right_layout = QVBoxLayout()

        # 로그 출력 영역 추가
        self.log_output = QTextEdit(self)
        self.log_output.setFixedSize(400, 600)

        self.log_output.setFont(QFont('Courier', 10))
        self.log_output.setStyleSheet("background-color: #f4f4f9; border: 1px solid #ccc;")
        self.log_output.setReadOnly(True)
        right_layout.addWidget(self.log_output)
        
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)
        self.show()



    def append_to_log(self, message):
        self.log_output.append(message)

    def increase_blue_box_count(self):
        self.blue_box_count += 1
        self.blue_box_count_label.setText(str(self.blue_box_count))

    def decrease_blue_box_count(self):
        if self.blue_box_count > 0:
            self.blue_box_count -= 1
            self.blue_box_count_label.setText(str(self.blue_box_count))

    def increase_red_box_count(self):
        self.red_box_count += 1
        self.red_box_count_label.setText(str(self.red_box_count))

    def decrease_red_box_count(self):
        if self.red_box_count > 0:
            self.red_box_count -= 1
            self.red_box_count_label.setText(str(self.red_box_count))

    def update_status(self, status):
        self.status_label.setText(f'현재 상태: {status}')

    def update_image(self, cv_image):
        if cv_image is not None:
            height, width, channels = cv_image.shape
            bytes_per_line = channels * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap(qt_image)
            self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), aspectRatioMode=1))
        else:
            self.image_label.setText('이미지가 없습니다.')

    def start_publishing(self):
        selected_job = self.job_combo.currentText()
        if selected_job == "Job1":
            selected_job_num = 1
        elif selected_job == "Job2":
            selected_job_num = 2
        elif selected_job == "Job3":
            selected_job_num = 3          
        blue_count = self.blue_box_count
        red_count = self.red_box_count

        data = {
            "red": red_count,
            "blue": blue_count,
            "goal" : selected_job_num
        }

        msg = json.dumps(data)
        self.ros_node.publish_message(msg)

    def conv_publishing(self):
        data = {
            "control": "go",
            "distance.mm": "100",
        }
        msg = json.dumps(data)
        self.ros_node.publish_conv_message(msg)

    def conv_stop_publishing(self):
        data = {
            "control": "stop",
        }
        msg = json.dumps(data)
        self.ros_node.publish_conv_message(msg)

    def closeEvent(self, event):
        rclpy.shutdown()

def main():
    app = QApplication(sys.argv)
    window = MyApp()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()