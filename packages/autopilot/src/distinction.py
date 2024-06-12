#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from sensor_msgs.msg import Range  # Import Range message for ToF sensor

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.stop_sign_distance_threshold = 0.2  # Set your desired threshold distance here (in meters)
        self.object_detection_min = 0.3  # Minimum distance threshold for object detection
        self.object_detection_max = 0.5  # Maximum distance threshold for object detection

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

        rospy.spin()  # Spin forever but listen to message callbacks

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        rospy.loginfo("AprilTag detections received.")
        self.move_robot(msg.detections)
 
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        rospy.loginfo("Sending stop command to robot.")
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        rospy.loginfo(f"Setting state to {state}.")
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            rospy.loginfo("No AprilTags detected.")
            return
        
        for detection in detections:
            rospy.loginfo(f"Detected AprilTag with ID: {detection.tag_id}")
            if detection.tag_id == 32:  # Assuming tag ID 32 is the stop sign
                distance_to_tag = detection.transform.translation.z
                rospy.loginfo(f"Distance to AprilTag (ID 32): {distance_to_tag} meters")
                if distance_to_tag <= self.stop_sign_distance_threshold:
                    rospy.loginfo("Stop sign within threshold distance. Stopping the robot...")
                    
                    # Change state to stop lane following
                    self.set_state("NORMAL_JOYSTICK_CONTROL")
                    
                    # Stop the robot
                    self.stop_robot()
                    rospy.sleep(3)  # Stop for 3 seconds

                    # Move forward to clear the stop sign
                    cmd_msg = Twist2DStamped()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.v = 0.5  # Move forward with velocity 0.5
                    cmd_msg.omega = 0.0
                    self.cmd_vel_pub.publish(cmd_msg)
                    rospy.sleep(2)  # Move forward for 2 seconds

                    # Stop the robot again
                    self.stop_robot()

                    # Ignore AprilTag messages for 5 seconds to avoid immediate re-triggering
                    self.ignore_apriltag = True
                    rospy.sleep(5)
                    self.ignore_apriltag = False

                    # Resume lane following
                    self.set_state("LANE_FOLLOWING")
                    rospy.loginfo("Resuming lane following...")

                    return

    def range_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        # Check if the distance is within the desired range for object detection
        if self.object_detection_min <= msg.range <= self.object_detection_max:
            # Object detected in front, stop the robot
            self.stop_robot()

            # Implement logic for Car Following or Overtaking feature here
            # For example, you can adjust the robot's behavior based on the detected distance
            
            # Implement closed-loop movement here if necessary
            pass
        else:
            # No obstacle detected within the specified range, resume lane following
            self.set_state("LANE_FOLLOWING")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
