import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        rospy.init_node('encoder_publisher')


        # Initialize Pub/Subs
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
            rospy.sleep(1)  # Wait for a second for the node to be ready # Calibrate ticks per meter
            self.turn_robot(90)# Move the robot forward by 1 meter
            self.stop_robot() # Stop the robot if in joystick control mode

    def encoder_callback(self, msg):
        # Store initial ticks value upon receiving first encoder message
        rospy.loginfo("Ticks: %s", msg.data)
        
    def publish_encoder_data(self, degrees):
    # Create a WheelEncoderStamped message
        encoder_msg = WheelEncoderStamped()

        # Populate the message fields (e.g., header, data, resolution, type)
        encoder_msg.header.stamp = rospy.Time.now()  # Use current time as the message timestamp
        encoder_msg.header.frame_id = "oryx/right_wheel_axis"
        encoder_msg.data += degrees
        encoder_msg.resolution = 135
        encoder_msg.type = 1

        # Publish the message
        self.publisher.publish(encoder_msg)
        rospy.loginfo("Published encoder data")

        
    def add_encoder_turn(self, msg, value):
        rospy.loginfo("adding 90 to right encoder: %s", msg.data)
        msg.data += value

    # def calibrate_ticks_per_meter(self):
    #     rospy.loginfo("Calibrating ticks per meter...")
    #     rospy.sleep(3)  # Wait for a few seconds to calibrate
    #     self.ticks_per_meter = self.ticks_sum / self.distance_moved
    #     rospy.loginfo(f"Ticks per meter calibrated: {self.ticks_per_meter}")

    def move_straight(self, distance):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.4  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")
        rospy.sleep(distance / 0.4)

    def stop_robot(self):
        # Send zero velocities to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")
        
    def turn_robot(self, degrees):
        self.publish_encoder_data(degrees)
        rospy.loginfo(f"Turning Right by {degrees} degrees...")
        


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
