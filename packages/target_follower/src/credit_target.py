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
        self.max_omega = math.radians(4.0)  # Maximum angular velocity in radians
        self.min_omega = math.radians(2.0)  # Minimum angular velocity in radians
        self.max_linear_speed = 0.2  # Maximum linear speed in meters per second
        self.goal_distance_min = 0.2  # Minimum goal distance to the AprilTag in meters
        self.goal_distance_max = 0.4  # Maximum goal distance to the AprilTag in meters
        self.deadband = 0.1  # Deadband around zero angular velocity
        self.high_friction_factor = 2  # Factor to increase omega for high ground friction
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
            if tag_position.z < self.goal_distance_min or tag_position.z > self.goal_distance_max:  # If tag is out of desired distance range
                self.move_robot(tag_position)
            else:
                self.stop_robot()
        else:
            # No AprilTag detected
            rospy.loginfo("No AprilTag detected.")
            self.tag_visible = False
            self.keep_spinning()

    # Method to stop the robot completely
    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    # Method to make the robot spin continuously
    def keep_spinning(self):
        self.publish_cmd_vel(0.0, math.radians(2.0))  # Constant angular velocity for spinning

    # Method to move the robot to face the AprilTag at the desired position
    def move_robot(self, tag_position):
        # Calculate the angular velocity to center the AprilTag in the camera frame
        angle_to_tag = math.atan2(tag_position.x, tag_position.z)
        omega = self.calculate_omega(angle_to_tag)

        # Calculate the linear speed to maintain the goal distance
        distance_error = tag_position.z - self.goal_distance_max if tag_position.z > self.goal_distance_max else self.goal_distance_min - tag_position.z
        linear_speed = self.calculate_linear_speed(distance_error)

        # Publish the motion command to the robot
        self.publish_cmd_vel(linear_speed, omega)

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

        # Clamp omega within limits
        omega = max(min(omega, self.max_omega), -self.max_omega)

        return omega

    # Method to calculate linear speed based on distance error
    def calculate_linear_speed(self, distance_error):
        # Proportional control with scaling factor
        linear_speed = distance_error * 0.1

        # Clamp linear speed within limits
        linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)

        return linear_speed

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
