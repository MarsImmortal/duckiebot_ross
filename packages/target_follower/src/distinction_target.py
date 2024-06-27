#!/usr/bin/env python3 (testing for credentials)

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
        self.max_omega = math.radians(90.0)  # Maximum angular velocity in radians per second
        self.min_omega = math.radians(5.0)  # Minimum angular velocity in radians per second
        self.max_linear_speed = 1.0  # Maximum linear speed in meters per second
        self.goal_distance_min = 0.15  # Minimum goal distance to the AprilTag in meters
        self.goal_distance_max = 0.25  # Maximum goal distance to the AprilTag in meters
        self.deadband = math.radians(1.0)  # Deadband around zero angular velocity
        self.tag_visible = False  # Flag to indicate if AprilTag is visible
        self.closest_tag_id = None  # ID of the currently followed tag
        self.closest_tag_distance = float('inf')  # Distance to the closest tag

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # Find the closest tag
            closest_tag = min(msg.detections, key=lambda detection: detection.transform.translation.z)
            tag_position = closest_tag.transform.translation

            # Check if we need to switch to a new closest tag
            if closest_tag.transform.translation.z < self.closest_tag_distance or self.closest_tag_id != closest_tag.tag_id:
                self.closest_tag_id = closest_tag.tag_id
                self.closest_tag_distance = closest_tag.transform.translation.z
                rospy.loginfo("Switching to closest tag ID: %d at distance: %.2f", self.closest_tag_id, self.closest_tag_distance)

            self.tag_visible = True
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
        self.publish_cmd_vel(0.0, self.min_omega)  # Constant angular velocity for spinning

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
        omega = -angle_to_tag * 2.0  # Reversed and increased gain for better response

        # Apply deadband around zero angular velocity
        if abs(omega) < self.deadband:
            omega = 0.0

        # Clamp omega within limits
        omega = max(min(omega, self.max_omega), -self.max_omega)

        return omega

    # Method to calculate linear speed based on distance error
    def calculate_linear_speed(self, distance_error):
        # Proportional control with scaling factor
        linear_speed = distance_error * 2.0  # Increased gain for overcoming friction

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
