import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray
import math
import time
import signal
import sys

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)

        # Publisher and subscriber initialization
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Flag to indicate if AprilTag is detected
        self.tag_detected = False

        # Start the ROS loop
        rospy.spin()

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            self.tag_detected = True
            self.stop_robot()
        else:
            self.tag_detected = False
            self.adjust_rotation()

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()
        rospy.signal_shutdown("Shutdown requested")

    # Signal handler for SIGINT (Ctrl+C)
    def signal_handler(self, signal, frame):
        rospy.loginfo("Received shutdown signal (Ctrl+C)")
        self.clean_shutdown()

    # Method to stop the robot completely
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to make short adjustments in rotation
    def adjust_rotation(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0  # No forward movement
        
        # Set the desired angular velocity for spinning
        cmd_msg.omega = 2.0
        
        # Rotate 10 degrees (in radians)
        target_angle = math.radians(10)
        
        # Publish the command to make a 10-degree turn
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Pause for 0.5 seconds after every 10 degrees rotation
        time.sleep(0.5)

    # Method to continuously spin the robot
    def keep_spinning(self):
        if not self.tag_detected:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0  # No forward movement
            cmd_msg.omega = 2.0  # Constant angular velocity for spinning
            self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
