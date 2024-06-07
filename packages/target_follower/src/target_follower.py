#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Publisher and subscriber initialization
        self.cmd_vel_pub = rospy.Publisher('/your_robot_name/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/your_robot_name/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Define control parameters
        self.max_omega = 2.0  # Maximum angular velocity
        self.min_omega = 0.5  # Minimum angular velocity
        self.tag_visible = False  # Flag to indicate if AprilTag is visible

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            self.tag_visible = True
            # Get the position of the first detected AprilTag
            tag_position = msg.detections[0].transform.translation
            rospy.loginfo("AprilTag position (x, y, z): (%.2f, %.2f, %.2f)", tag_position.x, tag_position.y, tag_position.z)
            # Move the robot to face the AprilTag
            self.move_robot(tag_position)
        else:
            self.tag_visible = False
            # Stop the robot if no AprilTag is detected
            self.stop_robot()

    # Method to stop the robot completely
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to move the robot to face the AprilTag
    def move_robot(self, tag_position):
        # Calculate the rotation angle based on the AprilTag position
        angle_to_tag = -tag_position.y  # Adjust direction based on camera orientation
        # Apply control algorithm
        self.rotate_robot(angle_to_tag)

    # Method to rotate the robot towards the AprilTag
    def rotate_robot(self, angle_to_tag):
        # Calculate the angular velocity based on the error term
        omega = angle_to_tag * 0.5  # Proportional control with scaling factor
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
