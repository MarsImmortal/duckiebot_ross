import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Publishers and Subscribers
        self.publisher = rospy.Publisher('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, queue_size=10)
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
            self.turn_robot(90)  # Turn the robot by 90 degrees
            self.stop_robot()  # Stop the robot after turning

    def encoder_callback(self, msg):
        # Store initial ticks value upon receiving first encoder message
        rospy.loginfo("Ticks: %s", msg.data)

    def publish_encoder_data(self, degree, msg):
        # Create a copy of the received message to modify
        encoder_msg = msg
        # Calculate the target degree
        target_degree = msg.data + degree
        # Update the message data with the new target degree
        encoder_msg.data = target_degree
        # Publish the modified message
        self.publisher.publish(encoder_msg)
        rospy.loginfo(f"Published encoder data: {target_degree}")

    def stop_robot(self):
        # Send zero velocities to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def turn_robot(self, degrees):
        # Publish the desired degree change
        rospy.loginfo(f"Turning Right by {degrees} degrees...")
        self.publish_encoder_data(degrees)

    def run(self):
        rospy.spin()  # Keeps the node from exiting until shutdown

if __name__ == '__main__':
    try:
        # Create an instance of Drive_Square class
        duckiebot_movement = Drive_Square()
        # Run the ROS node
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
