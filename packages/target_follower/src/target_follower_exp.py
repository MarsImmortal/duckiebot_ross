#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Subscriber initialization
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # Get the position of the first detected AprilTag
            tag_position = msg.detections[0].transform.translation
            rospy.loginfo("AprilTag position (x, y, z): (%.2f, %.2f, %.2f)", tag_position.x, tag_position.y, tag_position.z)
        else:
            rospy.loginfo("No AprilTag detected.")

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down.")

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
