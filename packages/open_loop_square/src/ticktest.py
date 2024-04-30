import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.ticks_per_meter = 561  # Ticks per meter (experimental value)
        self.ticks_per_90_degrees = 90  # Ticks per 90-degree turn (experimental value)

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)

        # Check the FSM state and perform actions accordingly
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()  # Stop the robot if in joystick control mode
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a second for the node to be ready
            self.move_straight(1.0)  # Move the robot forward by 1 meter
            self.rotate_in_place(90)  # Rotate the robot 90 degrees

    def encoder_callback(self, msg):
        # This function is not used for calibration in this case
        return msg.data

    def move_straight(self, distance):
        target_ticks = self.encoder_callback() + int(distance * self.ticks_per_meter)

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.4  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_ticks = self.encoder_callback()  # Get current ticks
            if current_ticks >= target_ticks:
                break
            rate.sleep()

        self.stop_robot()

    def rotate_in_place(self, degrees):
        target_ticks = self.encoder_callback() + int(degrees / 90 * self.ticks_per_90_degrees)

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5  # Angular velocity for a 90-degree turn (adjust as needed)
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Rotating in place by {degrees} degrees...")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_ticks = self.encoder_callback()
            if current_ticks >= target_ticks:
                break
            rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
