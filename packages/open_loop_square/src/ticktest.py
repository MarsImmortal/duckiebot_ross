import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.encoder_ticks = 0
        self.target_ticks = 0

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

    def encoder_callback(self, msg):
        self.encoder_ticks = msg.data

    def move_forward_one_meter(self):
        # Calculate target ticks for moving forward one meter
        ticks_per_meter = 135  # Example resolution (adjust based on your system)
        target_ticks = self.encoder_ticks + ticks_per_meter

        # Send velocity command to move forward
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.6  # Adjust linear velocity as needed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving forward...")

        # Monitor encoder ticks until reaching the target ticks (one meter)
        while not rospy.is_shutdown() and self.encoder_ticks < target_ticks:
            rospy.sleep(0.1)

        # Stop the robot after reaching the target distance
        self.stop_robot()

    def stop_robot(self):
        # Send zero velocity command to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def run(self):
        # Perform the forward movement and wait for completion
        self.move_forward_one_meter()

        # Spin to keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
