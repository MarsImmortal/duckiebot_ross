#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray
import math

class Target_Follower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Publisher and subscriber initialization
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Start spinning the robot
        self.keep_spinning()

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # Get the first detected AprilTag
            detection = msg.detections[0]
            
            # Calculate the angle between the robot's current heading and the AprilTag
            angle_to_tag = math.atan2(detection.transform.translation.y, detection.transform.translation.x)
            
            # Rotate the robot by the calculated angle
            self.rotate_robot(angle_to_tag)
        else:
            # No AprilTag detected, continue spinning
            self.keep_spinning()

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Method to stop the robot completely
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to continuously spin the robot
    def keep_spinning(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0  # No forward movement
        cmd_msg.omega = 1.0  # Constant angular velocity for spinning
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to rotate the robot by a specified angle
    def rotate_robot(self, angle):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0  # No forward movement
        cmd_msg.omega = angle  # Set the angular velocity to the calculated angle
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
