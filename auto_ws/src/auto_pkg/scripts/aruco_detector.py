#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image

# OpenCV
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher_ids = self.create_publisher(Int32MultiArray, 'detected_ids', 10)
        self.publisher_frame = self.create_publisher(Image, 'detected_frame', 10)
        self.publisher_corners = self.create_publisher(Float32MultiArray, 'detected_corners', 10)
        
        self.bridge = CvBridge()
        
        # Load image from parameter
        self.declare_parameter('image_path', '../marker_1.png') 
        self.image_path = self.get_parameter('image_path').value
        self.declare_parameter('camera_ip', '12')
        self.camera_ip = str(self.get_parameter('camera_ip').get_parameter_value().string_value)

        # ArUco predefined dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        VIDEOS_DIR = os.path.join('.', 'videos')
        video_path = os.path.join(VIDEOS_DIR, 'IMG_9135.mp4')
        video_path_out = '{}_out.mp4'.format(video_path)

        self.out = cv2.VideoWriter(video_path_out, fourcc, 20.0, (640,  480))
        
        # self.detect_aruco()
        
        # Pose estimation (dummy camera params for now)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)
        self.marker_length = 0.05
        
        # Capture video frames using rtsp
        rtsp_url = f"rtsp://admin:123456@192.168.1.11:554/mpeg4"
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open RTSP stream.")
        
        self.frame = None
        self.running = True

        # Start a thread to grab the latest frame
        self.thread = threading.Thread(target=self.update_frame, daemon=True)
        self.thread.start()

        self.get_logger().info("Aruco detection node has been started.")
        self.timer = self.create_timer(0.1, self.detect_aruco)  # Timer to run at 10 Hz

    def update_frame(self):
        # Continuously read frames to reduce RTSP latency
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame  # Store the latest frame
        
    def detect_aruco(self):
        if self.frame is None:
            self.get_logger().warning("Failed to capture frame from camera.")
            return
        
        frame = self.frame.copy()
        
        # Resize to half size
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

        # Add a white border (quiet zone)
        quiet_zone_size = 20
        image = cv2.copyMakeBorder(
            frame,
            quiet_zone_size,
            quiet_zone_size,
            quiet_zone_size,
            quiet_zone_size,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )

        # -------------------------------------------------------------------
        #       DETECT MARKER
        # -------------------------------------------------------------------

        # Detect markers
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
        # cv2.imshow("Aruco Detection", image)
        # cv2.waitKey(1)
        self.out.write(image)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

            
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()