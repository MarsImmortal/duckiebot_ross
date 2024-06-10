import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.ticks_per_meter = 561  # Ticks per meter (experimental value)
        self.current_ticks = 0
        self.start_ticks = 0
        self.obstacle_threshold = 0.3
        self.obstacle_detected = False

        rospy.init_node('drive_square_node', anonymous=True)
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.loginfo("Executing Lane Following Mode...")
            self.move_square()

    def encoder_callback(self, msg):
        self.current_ticks = msg.data

    def range_callback(self, msg):  
        # Validate the range data
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            if msg.range < self.obstacle_threshold:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def move_straight(self, distance):
        self.start_ticks = self.current_ticks
        target_ticks = self.start_ticks + int(distance * self.ticks_per_meter)
        rate = rospy.Rate(10)

        while self.current_ticks < target_ticks:
            if self.obstacle_detected:
                self.stop_robot()
                self.rotate_to_clear_obstacle()  # Rotate to find a clear path
                self.move_square()  # Restart the square movement
                return
            else:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.3  # Forward velocity (adjust as needed)
                self.cmd_msg.omega = 0.0
                self.pub.publish(self.cmd_msg)
                rate.sleep()

        self.stop_robot()

    def rotate_to_clear_obstacle(self):
        rate = rospy.Rate(10)
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 1.0  # Angular velocity (adjust as needed)

        for _ in range(30):  # Rotate for a fixed duration to clear the obstacle
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()

    def move_square(self):
        for _ in range(4):
            self.move_straight(1.0)
            self.rotate_in_place(90)

    def rotate_in_place(self, degrees):
        self.start_ticks = self.current_ticks
        target_ticks = self.start_ticks + int(degrees * self.ticks_per_meter / 360)  # Adjust for rotational ticks
        rate = rospy.Rate(10)

        while self.current_ticks < target_ticks:
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 1.0  # Angular velocity (adjust as needed)
            self.pub.publish(self.cmd_msg)
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
        rospy.loginfo("Drive Square Node Initialized...")
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Drive Square Node Interrupted...")
        pass
