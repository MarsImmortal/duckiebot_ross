#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class Autopilot:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.tof_distance = None
        self.tof_threshold = 0.5  # Threshold distance in meters for car following
        self.enc_left = 0
        self.enc_right = 0

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/akandb/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/akandb/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/akandb/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/akandb/front_center_tof_driver_node/range', Range, self.tof_callback, queue_size=1)
        rospy.Subscriber('/akandb/encoder_node/odom', Odometry, self.odom_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        self.move_robot(msg.detections)
    
    # ToF sensor callback
    def tof_callback(self, msg):
        self.tof_distance = msg.range
        self.check_tof_distance()
 
    # Encoder callback
    def odom_callback(self, msg):
        self.enc_left = msg.pose.pose.position.x  # Assuming x represents the left encoder ticks
        self.enc_right = msg.pose.pose.position.y  # Assuming y represents the right encoder ticks

    # Stop Robot before node has shut down. This ensures the robot doesn't keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
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

    def check_tof_distance(self):
        if self.tof_distance is not None and self.tof_distance < self.tof_threshold:
            rospy.loginfo("Object detected in front. Stopping the robot...")
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            self.stop_robot()
            rospy.sleep(3)  # Stop for 3 seconds

            rospy.loginfo("Overtaking the object...")
            self.overtake_object()

            self.set_state("LANE_FOLLOWING")
            rospy.loginfo("Resuming lane following...")

    def overtake_object(self):
        # Example of simple overtaking maneuver
        # Move left to overtake
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.5
        cmd_msg.omega = 1.0  # Turn left
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Turn for 2 seconds

        # Move forward
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Move forward for 2 seconds

        # Move right to get back to lane
        cmd_msg.omega = -1.0  # Turn right
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Turn for 2 seconds

        # Move forward to ensure we are back in the lane
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Move forward for 2 seconds

        self.stop_robot()

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        for detection in detections:
            if detection.id == 1:  # Assuming tag ID 1 is the stop sign
                rospy.loginfo("Stop sign detected. Stopping the robot...")
                self.set_state("NORMAL_JOYSTICK_CONTROL") # Stop Lane Following
                self.stop_robot()
                rospy.sleep(3)  # Stop for 3 seconds

                # Move forward a bit to ensure the stop sign is out of view
                cmd_msg = Twist2DStamped()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.v = 0.5  # Move forward with a velocity of 0.5
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
            
            elif detection.id == 2:  # Assuming tag ID 2 is the left turn sign
                rospy.loginfo("Left turn sign detected. Preparing to turn left...")
                self.handle_intersection("left")
                return
            
            elif detection.id == 3:  # Assuming tag ID 3 is the right turn sign
                rospy.loginfo("Right turn sign detected. Preparing to turn right...")
                self.handle_intersection("right")
                return

    def handle_intersection(self, direction):
        self.set_state("NORMAL_JOYSTICK_CONTROL") # Stop Lane Following
        self.stop_robot()
        
        # Move forward to a predefined position before turning
        self.move_forward_with_encoders(1.0)  # Move forward 1 meter

        if direction == "left":
            self.turn_left_with_encoders()
        elif direction == "right":
            self.turn_right_with_encoders()

        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Completed turn. Resuming lane following...")

    def move_forward_with_encoders(self, distance):
        # Example function to move forward a certain distance using encoders
        initial_left = self.enc_left
        initial_right = self.enc_right
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.5
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        while abs(self.enc_left - initial_left) < distance and abs(self.enc_right - initial_right) < distance:
            rospy.sleep(0.1)
        
        self.stop_robot()

    def turn_left_with_encoders(self):
        # Example function to turn left using encoders
        initial_left = self.enc_left
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 1.0  # Turn left
        self.cmd_vel_pub.publish(cmd_msg)

        while abs(self.enc_left - initial_left) < 0.5:  # Adjust this value based on your robot's turning capability
            rospy.sleep(0.1)

        self.stop_robot()

    def turn_right_with_encoders(self):
        # Example function to turn right using encoders
        initial_right = self.enc_right
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = -1.0  # Turn right
        self.cmd_vel_pub.publish(cmd_msg)

        while abs(self.enc_right - initial_right) < 0.5:  # Adjust this value based on your robot's turning capability
            rospy.sleep(0.1)

        self.stop_robot()

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
