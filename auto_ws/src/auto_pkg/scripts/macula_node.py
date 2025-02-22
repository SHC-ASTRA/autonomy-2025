#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from sensor_msgs.msg import Image

# OpenCV
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import os
from ultralytics import YOLO

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class MaculaNode(Node):
    def __init__(self):
        super().__init__('macula_node')

        self.publisher_ids = self.create_publisher(Int32MultiArray, 'detected_ids', 10)
        self.publisher_frame = self.create_publisher(Image, 'detected_frame', 10)
        self.publisher_corners = self.create_publisher(Float32MultiArray, 'detected_corners', 10)
        self.publisher_objects = self.create_publisher(Float32MultiArray, 'detected_objects', 10)

        mode = int(input("Enter 1 for ArUco Detection or 2 for Object Detection: "))
        if mode in [1, 2]:
            self.mode = mode
            self.get_logger().info(f"Mode set to {self.mode}")
        else:
            self.get_logger().error("Invalid input. Please enter 1 or 2.")
        
        self.bridge = CvBridge()

        # ArUco predefined dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Pose estimation (dummy camera params for now)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)
        self.marker_length = 0.05

        # Load YOLOv8 model
        self.model = YOLO("./src/auto_pkg/models/best.pt") 
        
        # Capture video frames using rtsp
        rtsp_url = f"rtsp://admin:123456@192.168.1.15:554/mpeg4"
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open RTSP stream.")
        
        self.frame = None
        self.running = True

        # Start a thread to grab the latest frame
        self.thread = threading.Thread(target=self.update_frame, daemon=True)
        self.thread.start()

        self.get_logger().info("Macula node has been started.")
        self.timer = self.create_timer(0.1, self.frame_mode)  # Timer to run at 10 Hz
    
    def update_frame(self):
        # Continuously read frames to reduce RTSP latency
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame  # Store the latest frame

    def frame_mode(self):
        if self.frame is None:
            return
        
        frame = self.frame.copy()
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

        if self.mode == 1:
            self.detect_aruco()
        elif self.mode == 2:
            self.detect_objects()

    def detect_aruco(self):
        # Detect markers
        image = self.frame
        corners, ids, _ = self.detector.detectMarkers(image)
        
        if ids is not None:
            self.get_logger().info(f"Detected ArUco IDs: {ids.flatten()}")
            self.get_logger().info(f"Detected ArUco corners: {corners[0]}")
            
            # Publish the detected IDs
            msg_ids = Int32MultiArray(data=ids.flatten())
            self.publisher_ids.publish(msg_ids)

            # Publish corners
            msg_corners = Float32MultiArray(data=corners[0].flatten().tolist())
            self.publisher_corners.publish(msg_corners)

            # Draw detected markers
            for i in range(len(ids)):
                # Draw borders around detected markers
                cv2.polylines(image, [np.int32(corners[i])], True, (255, 0, 0), 2)

                # Calculate the center of the marker
                mid_x = int(np.mean(corners[i][0][:, 0]))
                mid_y = int(np.mean(corners[i][0][:, 1]))

                cv2.putText(image, f"ID: {ids[i][0]}", (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                self.image = image

            # Publish the processed image
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.publisher_frame.publish(img_msg)
        else:
            self.get_logger().info("No markers detected.")
        # Show image
        cv2.imshow("Aruco Detection", image)
        cv2.waitKey(1)

    def detect_objects(self):
        frame = self.frame
        threshold = 0.05
        
        results = self.model(frame)[0]
        detected_objects = []

        for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result

                # Append to detected objects list
                detected_objects.extend([class_id, score])

                # Draw bounding box and confidence score on frame
                if score > threshold:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                    cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

        # Publish detected objects (Class ID, Confidence)
        if detected_objects:
            msg_objects = Float32MultiArray(data=detected_objects)
            self.publisher_objects.publish(msg_objects)

        # Show image
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

            
def main(args=None):
    rclpy.init(args=args)
    node = MaculaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
