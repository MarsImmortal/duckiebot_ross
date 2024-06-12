#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import WheelEncoderStamped

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.stop_sign_distance_threshold = 0.25  # Set your desired threshold distance here (in meters)
        self.encoder_ticks_per_meter = 1350  # Adjust according to your robot's specifications
        self.current_encoder_ticks = 0
        self.target_encoder_ticks = 0

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "oryx" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        rospy.loginfo("AprilTag detections received.")
        self.move_robot(msg.detections)
 
    # Encoder callback to update current encoder ticks
    def encoder_callback(self, msg):
        self.current_encoder_ticks = msg.data

    # Stop Robot before node has shut down. This ensures the robot does not keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
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
        
        # Process AprilTag info and publish a velocity
        for detection in detections:
            rospy.loginfo(f"Detected AprilTag with ID: {detection.tag_id}")
            if detection.tag_id == 57:  # Assuming tag ID 57 is the intersection sign
                distance_to_tag = detection.transform.translation.z
                rospy.loginfo(f"Distance to AprilTag (ID 57): {distance_to_tag} meters")
                if distance_to_tag <= self.stop_sign_distance_threshold:
                    rospy.loginfo("Intersection sign within threshold distance. Executing custom maneuver...")

                    # Change state to custom maneuver
                    self.set_state("CUSTOM_MANEUVER")

                    # Move forward 197 ticks
                    self.move_forward(197)

                    # Rotate by 325 ticks
                    self.rotate(-325)

                    # Move forward 622 ticks
                    self.move_forward(622)

                    # Resume lane following
                    self.set_state("LANE_FOLLOWING")
                    rospy.loginfo("Resuming lane following...")

                    return

    def move_forward(self, ticks):
        rospy.loginfo(f"Moving forward {ticks} encoder ticks.")
        initial_ticks = self.current_encoder_ticks
        target_ticks = initial_ticks + ticks

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.5  # Move forward with velocity 0.5
        cmd_msg.omega = 0.0

        while self.current_encoder_ticks < target_ticks:
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.01)  # Small sleep to prevent high CPU usage

        self.stop_robot()

    def rotate(self, ticks):
        rospy.loginfo(f"Rotating {ticks} encoder ticks.")
        initial_ticks = self.current_encoder_ticks
        target_ticks = initial_ticks + ticks

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 3  # Rotate with omega 3

        while self.current_encoder_ticks < target_ticks:
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.01)  # Small sleep to prevent high CPU usage

        self.stop_robot()

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
