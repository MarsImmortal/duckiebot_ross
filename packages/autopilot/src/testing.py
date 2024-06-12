#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.stop_sign_distance_threshold = 0.25  # Set your desired threshold distance here (in meters)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "oryx" WITH YOUR ROBOT'S NAME #####
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        rospy.loginfo("AprilTag detections received.")
        self.change_state_to_custom(msg.detections)

    # Stop Robot before node has shut down
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Cleaning up...")

    def set_state(self, state):
        rospy.loginfo(f"Setting state to {state}.")
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def change_state_to_custom(self, detections):
        if len(detections) == 0:
            rospy.loginfo("No AprilTags detected.")
            return
        
        # Process AprilTag info and change state if the tag is detected
        for detection in detections:
            rospy.loginfo(f"Detected AprilTag with ID: {detection.tag_id}")
            if detection.tag_id == 57:  # Assuming tag ID 57 is the intersection sign
                distance_to_tag = detection.transform.translation.z
                rospy.loginfo(f"Distance to AprilTag (ID 57): {distance_to_tag} meters")
                if distance_to_tag <= self.stop_sign_distance_threshold:
                    rospy.loginfo("Intersection sign within threshold distance. Changing state to CUSTOM_MANEUVER...")
                    self.set_state("CUSTOM_MANEUVER")
                    rospy.loginfo(f"Current state: {self.robot_state}")
                    return

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
