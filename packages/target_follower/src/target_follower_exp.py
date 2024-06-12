#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher for robot's velocity
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscriber initialization
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Start spinning
        self.start_spinning()
        
        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # Stop spinning when a tag is detected
            self.stop_spinning()

            # Get the position of the first detected AprilTag
            tag_position = msg.detections[0].transform.translation
            rospy.loginfo("AprilTag position (x, y, z): (%.2f, %.2f, %.2f)", tag_position.x, tag_position.y, tag_position.z)
        else:
            rospy.loginfo("No AprilTag detected.")
            # If no tag is detected, continue spinning
            self.start_spinning()

    # Function to start spinning the robot
    def start_spinning(self):
        twist = Twist()
        twist.angular.z = 0.5  # Adjust the angular speed as needed
        self.cmd_pub.publish(twist)

    # Function to stop the robot
    def stop_spinning(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping the robot.")
        self.stop_spinning()

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
