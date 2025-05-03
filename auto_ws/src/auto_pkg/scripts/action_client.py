#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from clucky_interfaces.action import AutoCommand

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image

# OpenCV
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import os
from ultralytics import YOLO

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class ActionClientNode(Node):
    def __init__(self):
        super().__init__("auto_action_client")
        self.declare_parameter('mission_type', '1') # Default to ArUco
        self.get_logger().info("Macula has been started.")
        
        # ActionClient(node, action_type, action_name *must be same as server)
        self.auto_action_client = ActionClient(self, AutoCommand, "auto_command") 
        
        self.publisher_ids = self.create_publisher(Int32MultiArray, 'detected_ids', 10)
        self.publisher_corners = self.create_publisher(Float32MultiArray, 'detected_corners', 10)
        self.publisher_objects = self.create_publisher(Float32MultiArray, 'detected_objects', 10)
        
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
        self.declare_parameter("camera_ip", 15) # Default to camera 12
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

        # Initialize VideoWriter for saving the video
        self.output_filename = 'stream.mp4'
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.fps = 30  # Adjust FPS as needed
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))  
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))   
        self.out = cv2.VideoWriter(self.output_filename, self.fourcc, self.fps, (self.frame_width, self.frame_height))
        
        self.timer = self.create_timer(0.1, self.frame_mode)  # Timer to run at 10 Hz
        
    def send_goal(self, mission_type):
        # Wait for action server
        self.auto_action_client.wait_for_server() # handle timeout
        
        # Create goal request
        self.goal = AutoCommand.Goal()
        self.goal.mission_type = mission_type
        self.goal.gps_lat_target = self.get_parameter('gps_lat_target').value
        self.goal.gps_long_target = self.get_parameter('gps_long_target').value
        self.goal.target_radius = self.get_parameter('target_radius').value
        self.goal.period = self.get_parameter('period').value
        
        # Send goal request
        self.get_logger().info("Sending goal from macula...")
        self.send_goal_future = self.auto_action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
            self.get_logger().info("Goal accepted by server.")
            
        else:
            self.get_logger().info("Goal rejected by server.")
            
    def goal_result_callback(self, future):
        result = future.result().result # final_result
        self.get_logger().info(f"Result: {result.final_result}")
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current status: {feedback.current_status}, Distance Remaining: {feedback.distance_remaining}")
        
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
        
        if self.goal.mission_type == 1:
            self.get_logger().info("Starting ArUco detection...")
            self.detect_aruco()
        elif self.goal.mission_type == 2:
            self.get_logger().info("Starting object detection...")
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
                # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs)
                # cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                self.image = image

        else:
            self.get_logger().info("No markers detected.")

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
                    cv2.putText(frame, str(round(score, 2)), (int(x2 - 10), int(y1)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
            
        
def main(args=None):
    mission_type = 1  # Default to ArUco detection
    
    rclpy.init(args=args)
    node = ActionClientNode()
    node.send_goal(mission_type)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
