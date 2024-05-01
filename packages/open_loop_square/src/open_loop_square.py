import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.ticks_per_meter = 561  # Ticks per meter (experimental value)
        self.ticks_per_90_degrees = 90  # Ticks per 90-degree turn (experimental value)
        self.current_ticks = 0
        self.obstacle_detected = False  # Flag for obstacle detection

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)

        # Check the FSM state and perform actions accordingly
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            rospy.loginfo("Switching to Normal Joystick Control Mode...")
            self.stop_robot()  # Stop the robot if in joystick control mode
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a second for the node to be ready
            rospy.loginfo("Executing Lane Following Mode...")
            self.make_square()  # Execute square pattern

    def encoder_callback(self, msg):
        # Update the current_ticks with the latest encoder value
        self.current_ticks = msg.data

    def range_callback(self, msg):
        # Check for obstacles within a defined range threshold
        obstacle_threshold = 0.2  # Adjust threshold as needed (in meters)

        if msg.range < obstacle_threshold:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle Detected!")
        else:
            self.obstacle_detected = False

        def move_straight(self, distance):
            target_ticks = self.current_ticks + int(distance * self.ticks_per_meter)

            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.6  # Forward velocity (adjust as needed)
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Moving Forward by {distance} meters...")

            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown() and self.current_ticks < target_ticks:
                if self.obstacle_detected:
                    rospy.loginfo("Obstacle Detected during Movement! Stopping...")
                    self.stop_robot()  # Stop the robot if obstacle detected
                    self.rotate_in_place(90)  # Rotate 90 degrees to avoid obstacle
                    self.move_straight(distance)  # Continue moving straight after obstacle avoidance
                    break
                rate.sleep()

        self.stop_robot()


    def rotate_in_place(self, degrees):
        target_ticks = self.current_ticks + int(degrees / 90 * self.ticks_per_90_degrees)

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 6  # Angular velocity for a 90-degree turn (adjust as needed)
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Rotating in place by {degrees} degrees...")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and self.current_ticks < target_ticks:
            rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def make_square(self):
        # Define side length of the square
        side_length = 0.5  # meters

        # Move forward and rotate 4 times to form a square
        for _ in range(4):
            rospy.loginfo("Moving Straight to Form Square...")
            self.move_straight(side_length)
            rospy.loginfo("Rotating to Form Square...")
            self.rotate_in_place(90)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        rospy.loginfo("Drive Square Node Initialized...")
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Drive Square Node Interrupted...")
        pass
