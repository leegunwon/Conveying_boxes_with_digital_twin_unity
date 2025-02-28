import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import pickle
import json  # JSON 처리를 위해 추가
from ultralytics import YOLO

PIXEL_PER_METER = 0.162

def get_calibrated_image(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    return dst

def draw_yellow_cross_line(img):
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (0, 255, 255), 3)
    cv2.line(img, (0, h // 2), (w, h // 2), (0, 255, 255), 3)
    return img

def detect_objects(model, img):
    """ 이미지에서 객체 감지 """
    results = model(img)  # 객체 감지 수행
    return img, results

def draw_results(img, results):
    """ 감지 결과 표시 및 중앙 거리 계산 """
    h, w = img.shape[:2]
    image_center = (w // 2, h // 2)  # 이미지 중앙 좌표
    detected_objects = []  # 감지된 객체 정보를 저장할 리스트

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0].item()
            label = result.names[int(box.cls[0])]

            # 박스 중앙 좌표 계산
            box_center = ((x1 + x2) // 2, (y1 + y2) // 2)

            # 박스 그리기
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f'{label} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # 중앙 거리 계산
            distance_x = -((image_center[0]) - np.array(box_center[0]))
            distance_y = (image_center[1]) - np.array(box_center[1])
            distance_x_meters = distance_x * PIXEL_PER_METER  # 픽셀 단위를 미터 단위로 변환
            distance_y_meters = distance_y * PIXEL_PER_METER

            # 거리 정보 표시
            cv2.putText(img, f'x :{distance_x_meters:.2f}m y :{distance_y_meters:.2f}mm', (box_center[0], box_center[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            if distance_x_meters < 0 and distance_y_meters > 0:
                screen_position = 0
            elif distance_x_meters > 0 and distance_y_meters > 0:
                screen_position = 1
            elif distance_x_meters < 0 and distance_y_meters < 0:
                screen_position = 2
            else:
                screen_position = 3
            
            # 감지된 객체 정보 저장
            detected_objects.append([int(box.cls[0]), distance_x_meters, distance_y_meters, screen_position])

    return img, detected_objects

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning
        self.model = YOLO('2nd_B1_best.pt')

        with open("/home/gunwon/serial_ws/src/processing_image/processing_image/calibrition.pkl", "rb") as f:
            self.mtx, self.dist = pickle.load(f)

        self.publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 10)
        self.info_publisher = self.create_publisher(String, 'yolo/detected_info', 10)  # String 메시지 퍼블리셔
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.image_np = None
        self.detected_objects = []  # 감지된 객체 정보 저장

    def timer_callback(self):
        if self.image_np is not None:
            dst = get_calibrated_image(self.image_np, self.mtx, self.dist)
            resized_image = cv2.resize(dst, (640, 360))
            dst_with_yellow_line = draw_yellow_cross_line(resized_image)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 압축 품질
            _, compressed_image = cv2.imencode('.jpg', dst_with_yellow_line, encode_param)

            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 설정
            compressed_msg.format = 'jpeg'  # 압축 포맷
            compressed_msg.data = compressed_image.tobytes()  # 압축된 데이터를 바이트 형식으로 저장

            self.publisher_.publish(compressed_msg)
            self.get_logger().info('Published compressed image.')

            # 감지된 객체 정보를 JSON 문자열로 변환하여 퍼블리시
            if self.detected_objects:
                json_str = json.dumps(self.detected_objects)  # 이중 리스트를 JSON 문자열로 변환
                info_msg = String()
                info_msg.data = json_str
                self.info_publisher.publish(info_msg)
                self.get_logger().info(f'Published detected objects info: {json_str}')

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        image, result = detect_objects(self.model, image)
        self.image_np, self.detected_objects = draw_results(image, result)  # 감지된 객체 정보 업데이트

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()