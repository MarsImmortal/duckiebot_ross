#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollowerFSM:
    def __init__(self):
        rospy.init_node('target_follower_fsm_node', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        self.rate = rospy.Rate(10)  # Control loop rate (Hz)

        self.Kp = 0.3  # Proportional gain for angular control

        self.target_detected = False
        self.tag_position_x = 0.0

        self.state = 'idle'
        self.run_fsm()

    def tag_callback(self, msg):
        if msg.detections:
            self.target_detected = True
            self.tag_position_x = msg.detections[0].transform.translation.x
        else:
            self.target_detected = False

    def run_fsm(self):
        while not rospy.is_shutdown():
            if self.state == 'idle':
                self.handle_idle_state()
            elif self.state == 'tracking':
                self.handle_tracking_state()
            elif self.state == 'lost':
                self.handle_lost_state()
            else:
                rospy.logerr("Invalid state!")

            self.rate.sleep()

    def handle_idle_state(self):
        if self.target_detected:
            rospy.loginfo("Tag detected. Transitioning to tracking state.")
            self.state = 'tracking'

    def handle_tracking_state(self):
        if not self.target_detected:
            rospy.loginfo("Tag lost. Transitioning to lost state.")
            self.state = 'lost'
            return

        # Calculate error
        target_x = 0.0  # Center of camera frame
        x_error = target_x - self.tag_position_x

        # Calculate angular velocity based on error
        omega = self.Kp * x_error

        # Publish velocity command
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # Linear velocity
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

    def handle_lost_state(self):
        if self.target_detected:
            rospy.loginfo("Tag found. Transitioning back to tracking state.")
            self.state = 'tracking'

        # Stop the robot
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        TargetFollowerFSM()
    except rospy.ROSInterruptException:
        pass
