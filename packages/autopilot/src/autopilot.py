#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray, WheelEncoderStamped
from duckietown_msgs.msg import AprilTagDetection

class Autopilot:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False

        # Register the clean_shutdown function to be called on shutdown
        rospy.on_shutdown(self.clean_shutdown)
        
        # Initialize Publishers
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)

        # Initialize Subscriber
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        # Spin to keep the script running and handle callbacks
        rospy.spin() 

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        # Only process if the robot is in LANE_FOLLOWING state and we are not ignoring AprilTags
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        # Process the detected AprilTags
        self.move_robot(msg.detections)
 
    # Clean shutdown function to stop the robot safely
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Function to send zero velocity command to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # Zero linear velocity
        cmd_msg.omega = 0.0  # Zero angular velocity
        self.cmd_vel_pub.publish(cmd_msg)

    # Function to set the robot's state and publish it
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    # Function to handle robot movement based on AprilTag detections
    def move_robot(self, detections):
        if len(detections) == 0:
            return

        # Loop through detected AprilTags
        for detection in detections:
            if detection.id == 32:  # Assuming tag ID 1 is the stop sign
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

# Main function
if __name__ == '__main__':
    try:
        # Create an instance of the Autopilot class and start the node
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        # Handle the case where the ROS node is interrupted
        pass
