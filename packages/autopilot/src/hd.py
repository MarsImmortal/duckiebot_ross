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
        self.object_detection_min = 0.15  # Minimum distance threshold for object detection
        self.object_detection_max = 0.2  # Maximum distance threshold for object detection

        self.ticks_per_90_degrees = 35  # Ticks per 90-degree turn (experimental value)
        self.current_ticks = 0
        self.start_ticks = 0  # Variable to track starting tick count
        self.obstacle_detected = False
        self.handling_obstacle = False  # Flag to indicate obstacle handling
        self.is_shutdown = False  # Flag to check if the node is shutting down

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
        
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot does not keep moving with the latest velocity command
    def clean_shutdown(self):
        self.is_shutdown = True
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        if not self.is_shutdown:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            return
        
        # Process AprilTag info and publish a velocity
        for detection in detections:
            if detection.tag_id == 31:  # Assuming tag ID 31 is the stop sign
                distance_to_tag = detection.transform.translation.z
                if distance_to_tag <= self.stop_sign_distance_threshold:
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
                    ignore_timer = rospy.Timer(rospy.Duration(5), self.reset_ignore_apriltag, oneshot=True)

                    # Resume lane following
                    self.set_state("LANE_FOLLOWING")

                    return

    def reset_ignore_apriltag(self, event):
        self.ignore_apriltag = False


    def range_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.handling_obstacle:
            return

        # Check if the distance is within the desired range for object detection
        if self.object_detection_min <= msg.range <= self.object_detection_max:
            self.obstacle_detected = True
            self.handling_obstacle = True  # Set flag to indicate obstacle handling
            self.set_state("CUSTOM_MANEUVER")
            self.avoid_obstacle()
        else:
            self.obstacle_detected = False

    def avoid_obstacle(self):
        # Perform left 90-degree turn
        self.turn_right()
        # Move forward for 60 ticks
        self.move_forward_ticks(70)

        # Resume lane following
        self.handling_obstacle = False  # Reset flag after handling obstacle
        self.set_state("LANE_FOLLOWING")

    def turn_left(self):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + self.ticks_per_90_degrees
        while self.current_ticks < target_ticks:
            if self.is_shutdown:
                return
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 3.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.2)  # Increased sleep duration to reduce CPU usage
        self.stop_robot()  # Ensure the robot stops after turning

    def turn_right(self):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + self.ticks_per_90_degrees
        while self.current_ticks < target_ticks:
            if self.is_shutdown:
                return
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = -3.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.2)  # Increased sleep duration to reduce CPU usage
        self.stop_robot()  # Ensure the robot stops after turning

    def move_forward_ticks(self, ticks):
        self.start_ticks = self.current_ticks  # Set starting ticks
        target_ticks = self.start_ticks + ticks
        while self.current_ticks < target_ticks:
            if self.is_shutdown:
                return
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.3
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(0.2)  # Increased sleep duration to reduce CPU usage
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
