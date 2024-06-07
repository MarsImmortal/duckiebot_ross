import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Setup shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
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
            tag_ids = [tag.id for tag in msg.detections]
            rospy.loginfo("AprilTag detected with IDs: %s" % tag_ids)
            self.stop_robot()
        else:
            self.tag_detected = False
            self.keep_spinning()

    # Stop the robot safely on shutdown
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Method to stop the robot completely
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Method to continuously spin the robot
    def keep_spinning(self):
        if not self.tag_detected:
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0  # No forward movement
            cmd_msg.omega = 1.0  # Constant angular velocity for spinning
            self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
