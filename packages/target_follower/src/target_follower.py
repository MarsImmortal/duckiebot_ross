#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray
import math

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Publisher and subscriber initialization
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Define control parameters
        self.max_omega = 2.0  # Maximum angular velocity
        self.min_omega = 0.5  # Minimum angular velocity
        self.tag_visible = False  # Flag to indicate if AprilTag is visible

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # AprilTag detected
            self.tag_visible = True
            tag_position = msg.detections[0].transform.translation
            rospy.loginfo("AprilTag position (x, y, z): (%.2f, %.2f, %.2f)", tag_position.x, tag_position.y, tag_position.z)
            if abs(tag_position.x) < 0.05:  # If tag is close to center
                self.stop_robot()
            else:
                self.move_robot(tag_position)
        else:
            # No AprilTag detected
            self.tag_visible = False
            self.keep_spinning()

    # Method to stop the robot completely
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to make the robot spin continuously
    def keep_spinning(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0  # No forward movement
        cmd_msg.omega = 1.0  # Constant angular velocity for spinning
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to move the robot to face the AprilTag at the desired position
    def move_robot(self, tag_position):
        # Desired x-coordinate of the AprilTag (center of the camera frame)
        desired_x = 0.0

        # Calculate the angle between the current position and the desired position
        angle_to_tag = math.atan2(desired_x - tag_position.x, tag_position.y)

        # Apply control algorithm
        self.rotate_robot(angle_to_tag)

    # Method to rotate the robot towards the AprilTag
    def rotate_robot(self, angle_to_tag):
        # Calculate the angular velocity based on the sign of the error term
        if angle_to_tag > 0:
            omega = -self.max_omega
        else:
            omega = self.max_omega

        # Apply minimum and maximum limits
        omega = max(self.min_omega, min(self.max_omega, omega))

        # Publish the rotation command to the robot
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # No forward movement
        cmd_msg.omega = omega  # Set the angular velocity
        self.cmd_vel_pub.publish(cmd_msg)

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
