#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from ros2_interfaces_pkg import msg

# OpenCV
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import os

# YOLO
from ultralytics import YOLO

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class MaculaNode(Node):
    def __init__(self):
        super().__init__('macula_node')
        self.get_logger().info("Macula Node has been started.")

        # Get mission type from parameter
        self.declare_parameter("detection_type", 1) # Default to ArUco
        self.detection_type = self.get_parameter("detection_type").get_parameter_value().integer_value
        
        # Create publisher
        self.macula_feedback = self.create_publisher(msg.MaculaFeedback, "/auto/macula", 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Capture video frames using rtsp
        self.declare_parameter("camera_ip", 12) # Default to camera 12
        camera_ip = self.get_parameter("camera_ip").get_parameter_value().integer_value
        rtsp_url = f"rtsp://admin:123456@192.168.1.{camera_ip}:554/mpeg4"
        self.get_logger().info(f"Connecting to RTSP stream: {rtsp_url}")

        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open RTSP stream.")
        
        self.frame = None
        self.running = True

        # Start a thread to grab the latest frame
        self.thread = threading.Thread(target=self.update_frame, daemon=True)
        self.thread.start()
        
        # ArUco predefined dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Load YOLOv8 model
        self.model = YOLO("./src/macula_pkg/models/best.pt") 
        
        # Get object detection threshold from parameter
        self.declare_parameter("confidence_threshold", 0.5)
        self.threshold = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        
        self.timer = self.create_timer(0.1, self.frame_mode)  # Timer to run at 10 Hz
        
    def update_frame(self):
        # Continuously read frames to reduce RTSP latency
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame  # Store the latest frame
    
    def frame_mode(self):
        self.frame = self.frame.copy()
        
        if self.detection_type == 1:
            self.get_logger().info("Starting ArUco detection...")
            self.detect_aruco()
        elif self.detection_type == 2:
            self.get_logger().info("Starting object detection...")
            self.detect_objects()
    
    def detect_aruco(self):
        # Detect markers
        image = self.frame
        corners, ids, _ = self.detector.detectMarkers(image)
        msg_out = msg.MaculaFeedback()
        
        if ids is not None:
            self.get_logger().info(f"Detected ArUco IDs: {ids.flatten()}")
            self.get_logger().info(f"Detected ArUco corners: {corners[0]}")
            
            for i in range(len(ids)):
                # Draw borders around detected markers
                cv2.polylines(image, [np.int32(corners[i])], True, (255, 0, 0), 2)

                # Calculate the center of the marker
                mid_x = int(np.mean(corners[i][0][:, 0]))
                mid_y = int(np.mean(corners[i][0][:, 1]))

                cv2.putText(image, f"ID: {ids[i][0]}", (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                # cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                self.image = image
                
                
                
                msg_out.detected = True
                msg_out.object_id = int(ids[0][0])
                msg_out.x0 = float(corners[0][0][0][0])
                msg_out.y0 = float(corners[0][0][0][1])
                msg_out.x1 = float(corners[0][0][1][0])
                msg_out.y1 = float(corners[0][0][1][1])
                msg_out.x2 = float(corners[0][0][2][0])
                msg_out.y2 = float(corners[0][0][2][1])
                msg_out.x3 = float(corners[0][0][3][0])
                msg_out.y3 = float(corners[0][0][3][1])
          
        else:
            self.get_logger().info("No markers detected.")
            msg_out.detected = False
        
        # Publish feedback message
        self.macula_feedback.publish(msg_out)
        
        # -------------------FOR TESTING-------------------
        frame = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)
        
    def detect_objects(self):
        frame = self.frame
        results = self.model(frame)[0]
        detected_objects = []
        
        msg_out = msg.MaculaFeedback()
        msg_out.detected = False
        
        for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result

                # Append to detected objects list
                detected_objects.extend([class_id, score])
                self.get_logger().info(f"Detected object: {class_id}, Confidence: {score}")
                
                if score > self.threshold:
                    msg_out.detected = True
                    
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                    cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
                    cv2.putText(frame, str(round(score, 2)), (int(x2 - 10), int(y1)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
                    
                # Set Object ID
                if class_id == 1:
                    msg_out.object_id = 51  # Mallet
                elif class_id == 0:
                    msg_out.object_id = 52 # Bottle
                    
                # Corners
                msg_out.x0 = float(x1)
                msg_out.y0 = float(y1)
                msg_out.x1 = float(x2)
                msg_out.y1 = float(y1)
                msg_out.x2 = float(x2)
                msg_out.y2 = float(y2)
                msg_out.x3 = float(x1)
                msg_out.y3 = float(y2)
                
        # Publish feedback message
        self.macula_feedback.publish(msg_out)
        
        # ----------------------FOR DEBUG----------------------
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("Object Detection", frame)
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