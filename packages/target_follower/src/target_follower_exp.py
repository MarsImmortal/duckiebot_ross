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
        
        # # Define control parameters
        # self.max_omega = math.radians(4.0)  # Maximum angular velocity in radians
        # self.min_omega = math.radians(2.0)  # Minimum angular velocity in radians
        # self.deadband = 0.5 # Deadband around zero angular velocity
        # self.high_friction_factor = 3  # Factor to increase omega for high ground friction
        # self.tag_visible = False  # Flag to indicate if AprilTag is visible

        # Start spinning until AprilTag is detected
        self.keep_spinning()

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # AprilTag detected
            self.tag_visible = True
            tag_position = msg.detections[0].transform.translation
            rospy.loginfo("AprilTag position (x, y, z): (%.2f, %.2f, %.2f)", tag_position.x, tag_position.y, tag_position.z)
            if abs(tag_position.x) < 0.10:  # If tag is close to center
                self.stop_robot()
            else:
                self.move_robot(tag_position)
        else:
            # No AprilTag detected
            rospy.loginfo("No AprilTag detected.")
            self.tag_visible = False

    # Method to stop the robot completely
    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    # Method to make the robot spin continuously
    def keep_spinning(self):
        while not self.tag_visible:
            self.publish_cmd_vel(0.0, math.radians(2.0))  # Constant angular velocity for spinning
            rospy.sleep(0.1)  # Add a small delay to avoid consuming too much CPU

    # Method to move the robot to face the AprilTag at the desired position
    def move_robot(self, tag_position):
        # Desired x-coordinate of the AprilTag (center of the camera frame)
        desired_x = 0.0

        # Calculate the angle between the current position and the desired position
        angle_to_tag = math.atan2(desired_x - tag_position.x, tag_position.y)

        # Adjust sign and magnitude of omega based on the angle
        omega = self.calculate_omega(angle_to_tag)

        # Publish the rotation command to the robot
        self.publish_cmd_vel(0.0, omega)

    # Method to calculate omega based on the angle
    def calculate_omega(self, angle_to_tag):
        # Proportional control with scaling factor
        omega = angle_to_tag * 0.5

        # Adjust omega for high ground friction and opposite rotations
        if omega < 0:
            omega *= self.high_friction_factor

        # Apply deadband around zero angular velocity
        if abs(omega) < self.deadband:
            omega = 0.0

        return omega

    # Method to publish Twist2DStamped message
    def publish_cmd_vel(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = v  # Linear velocity
        cmd_msg.omega = omega  # Angular velocity
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