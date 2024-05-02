#!/usr/bin/env python3

import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

class LaneDetector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        rospy.init_node("my_lane_detector")  # Initialize ROS node

        # Subscribe to the image topic (change topic name accordingly)
        self.image_sub = rospy.Subscriber('/akandb/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        rospy.loginfo("Image received")

        # Convert ROS image message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Crop the image to focus on the road region (adjust crop coordinates as needed)
        height, width, _ = img.shape
        roi_top = int(height * 0.8)
        roi_bottom = height
        roi_left = int(width * 0.8)
        roi_right = int(width * 0.8)
        cropped_img = img[roi_top:roi_bottom, roi_left:roi_right]

        # Convert cropped image to HSV color space
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Define white and yellow color ranges in HSV
        white_lower = np.array([0, 0, 200], dtype=np.uint8)
        white_upper = np.array([180, 30, 255], dtype=np.uint8)
        yellow_lower = np.array([20, 100, 100], dtype=np.uint8)
        yellow_upper = np.array([40, 255, 255], dtype=np.uint8)

        # Create masks for white and yellow color ranges
        white_mask = cv2.inRange(hsv_img, white_lower, white_upper)
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)

        # Apply Canny edge detection on the grayscale version of the cropped image
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_img, threshold1=50, threshold2=150)

        # Apply Hough transform to detect lines in the white-masked image
        white_lines = cv2.HoughLinesP(white_mask, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=10)
        
        # Apply Hough transform to detect lines in the yellow-masked image
        yellow_lines = cv2.HoughLinesP(yellow_mask, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=10)

        # Draw detected lines on the original cropped image
        if white_lines is not None:
            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cropped_img, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if yellow_lines is not None:
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cropped_img, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Display the processed images in separate windows
        cv2.imshow('Lane Detection - Cropped Image', cropped_img)
        cv2.imshow('Lane Detection - White Mask', white_mask)
        cv2.imshow('Lane Detection - Yellow Mask', yellow_mask)
        cv2.imshow('Canny Edge Detection', edges)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()  # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = LaneDetector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
