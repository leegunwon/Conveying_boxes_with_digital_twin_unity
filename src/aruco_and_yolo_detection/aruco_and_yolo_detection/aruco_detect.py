#!/usr/bin/env python3
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from aruco_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                distance = np.linalg.norm(tvec)
                detect_data.append([marker_id, marker_pos, (yaw, pitch, roll), distance])
    return image, detect_data

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

def load_camera_parameters(yaml_file):
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)

    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
        
    
    return camera_matrix, dist_coeffs
    

def get_calibrated_image(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    return dst


class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)

        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        
        with open("src/aruco_and_yolo_detection/aruco_and_yolo_detection/calibrition.pkl", "rb") as f:
            self.mtx, self.dist = pickle.load(f)

        self.bridge = CvBridge()
        self.marker_size = 0.04
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        
        # self.camera_matrix  = []
        # self.dist_coeffs = []

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        dst = get_calibrated_image(frame, self.mtx, self.dist)
        # width, height, channels = frame.shape
        # self.camera_matrix  = np.array([[360, 0, width/2], [0, 360, height/2], [0, 0, 1]])
        # self.dist_coeffs =  np.array([[0, 0, 0, 0, 0]])
        
        frame, detect_data = detect_markers(dst, self.camera_matrix, self.dist_coeffs, self.marker_size)
        if len(detect_data) == 0:
            self.get_logger().debug("No markers detected")
        else:
            closest_marker = min(detect_data, key=lambda x: x[3])
            self.get_logger().debug(f"Closest Marker ID: {closest_marker[0]}, Distance: {closest_marker[3]:.2f}m")

            marker_array_msg = MarkerArray()
            for marker in detect_data:
                marker_msg = Marker()
                marker_msg.id = int(marker[0])
                marker_msg.pose.pose.position.x = marker[1][0]
                marker_msg.pose.pose.position.y = marker[1][1]
                marker_msg.pose.pose.position.z = marker[1][2]
                marker_msg.pose.pose.orientation.x = marker[2][2]
                marker_msg.pose.pose.orientation.y = marker[2][1]
                marker_msg.pose.pose.orientation.z = marker[2][0]
                marker_array_msg.markers.append(marker_msg)
            self.marker_publisher.publish(marker_array_msg)
        # cv2.imshow('Detected Markers', frame)
        # cv2.waitKey(1)

def main(args=None):
    parser = argparse.ArgumentParser(description='Detect ArUco markers.')
    parser.add_argument('--marker_size', type=float, default=0.04,
                        help='Size of the ArUco markers in meters.')
    args = parser.parse_args()
    ArucoMarkerDetector.marker_size = args.marker_size
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()