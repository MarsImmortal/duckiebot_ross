#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray, WheelEncoderStamped
from sensor_msgs.msg import Range  # Import Range message for obstacle detection

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.stop_sign_distance_threshold = 0.2  # Set your desired threshold distance here (in meters)
        self.object_detection_min = 0.2  # Minimum distance threshold for object detection
        self.object_detection_max = 0.4  # Maximum distance threshold for object detection

        self.ticks_per_90_degrees = 100  # Ticks per 90-degree turn (experimental value)
        self.current_ticks = 0
        self.start_ticks = 0  # New variable to track starting tick count
        self.obstacle_detected = False
        self.handling_obstacle = False  # Flag to indicate obstacle handling

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "oryx" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)  # Subscribe to ToF sensor
        ################################################################

        rospy.spin()  # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag or self.handling_obstacle:
            return
        
        rospy.loginfo("AprilTag detections received.")
        self.move_robot(msg.detections)
 
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
            if detection.tag_id == 31:  # Assuming tag ID 31 is the stop sign
                distance_to_tag = detection.transform.translation.z
                rospy.loginfo(f"Distance to AprilTag (ID 31): {distance_to_tag} meters")
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
        if self.robot_state != "LANE_FOLLOWING" or self.handling_obstacle:
            return

        # Check if the distance is within the desired range for object detection
        if self.object_detection_min <= msg.range <= self.object_detection_max:
            rospy.loginfo(f"Obstacle detected at {msg.range} meters. Initiating avoidance maneuver...")
            self.obstacle_detected = True
            self.handling_obstacle = True  # Set flag to indicate obstacle handling
            self.set_state("CUSTOM_MANEUVER")
            self.avoid_obstacle()
        else:
            self.obstacle_detected = False

    def avoid_obstacle(self):
        rate = rospy.Rate(10)  # 10 Hz

        # Perform left 90-degree turn
        rospy.loginfo("Performing left 90-degree turn.")
        self.turn_left()

        # Move forward for 40 ticks
        rospy.loginfo("Moving forward for 40 ticks.")
        self.move_forward_ticks(40)

        # Perform right 90-degree turn
        rospy.loginfo("Performing right 90-degree turn.")
        self.turn_right()

        # Resume lane following
        rospy.loginfo("Resuming lane following...")
        self.handling_obstacle = False  # Reset flag after handling obstacle
        self.set_state("LANE_FOLLOWING")

    def turn_left(self):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + self.ticks_per_90_degrees
        while self.current_ticks < target_ticks:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 3.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.1)
        self.stop_robot()  # Ensure the robot stops after turning

    def turn_right(self):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + self.ticks_per_90_degrees
        while self.current_ticks < target_ticks:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = -3.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.1)
        self.stop_robot()  # Ensure the robot stops after turning

    def move_forward_ticks(self, ticks):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + ticks
        while self.current_ticks < target_ticks:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.3
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.1)
        self.stop_robot()  # Ensure the robot stops after moving forward

    # Wheel Encoder Callback
    def encoder_callback(self, msg):
        # Update the current ticks
        self.current_ticks = msg.data

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
