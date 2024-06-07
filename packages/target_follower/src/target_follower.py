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
            
            # Set the desired angular velocity for spinning
            cmd_msg.omega = 1.0  # Constant angular velocity for spinning
            
            # Publish the command to start spinning
            self.cmd_vel_pub.publish(cmd_msg)

            # Wait for 1 second
            rospy.sleep(1.0)

            # Calculate the target angle (in radians) for a 20-degree turn
            target_angle = math.radians(20)

            # Calculate the angular velocity required to achieve a 20-degree turn in 1 second
            # Angular velocity (omega) = Angle (in radians) / Time (in seconds)
            angular_velocity = target_angle / 1.0  # 1 second for a 20-degree turn
            
            # Update the angular velocity in the command message
            cmd_msg.omega = angular_velocity

            # Publish the command to make the robot spin 20 degrees
            self.cmd_vel_pub.publish(cmd_msg)

            # Wait for 1 second (to complete the 20-degree turn)
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass
