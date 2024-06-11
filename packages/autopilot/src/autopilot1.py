#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/akandb/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/akandb/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/akandb/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
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

    def move_robot(self, detections):
        if len(detections) == 0:
            return
        
        # Process AprilTag info and publish a velocity
        for detection in detections:
            if detection.tag_id == 32:  # Assuming tag ID 32 is the stop sign
                rospy.loginfo("Stop sign detected. Stopping the robot...")
                
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

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
