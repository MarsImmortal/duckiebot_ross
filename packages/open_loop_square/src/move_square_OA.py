import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.ticks_per_meter = 561  # Ticks per meter (experimental value)
        self.current_ticks = 0
        self.start_ticks = 0
        self.min_range = 0.05  # Minimum range for obstacle detection (in meters)
        self.max_range = 1.2  # Maximum range for obstacle detection (in meters)
        self.obstacle_threshold = 0.3
        self.obstacle_detected = False
        self.moving_forward = False

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
        if msg.range >= self.min_range and msg.range <= self.max_range:
            if msg.range < self.obstacle_threshold:
                if not self.obstacle_detected:  # Only update if obstacle status changed
                    self.obstacle_detected = True
                    if self.moving_forward:
                        self.stop_robot()
                        self.moving_forward = False
            else:
                if self.obstacle_detected:  # Only update if obstacle status changed
                    self.obstacle_detected = False
                    if not self.moving_forward:
                        self.start_ticks = self.current_ticks
                        self.moving_forward = True

    def move_straight(self, distance):
        target_ticks = self.start_ticks + int(distance * self.ticks_per_meter)
        rate = rospy.Rate(10)

        while self.current_ticks < target_ticks:
            if self.obstacle_detected:
                self.stop_robot()
                return
            else:
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.3  # Forward velocity (adjust as needed)
                self.cmd_msg.omega = 0.0
                self.pub.publish(self.cmd_msg)
                rate.sleep()

    def move_square(self):
        for _ in range(4):
            self.move_straight(1.0)
            if self.obstacle_detected:  # Check if an obstacle is detected while moving
                while self.obstacle_detected:  # Wait until the obstacle is removed
                    rospy.sleep(1)  # Check every second if the obstacle is removed
            self.rotate_in_place(90)

    def rotate_in_place(self, degrees):
        target_ticks = self.current_ticks + int(degrees * self.ticks_per_meter / 360)  # Adjust for rotational ticks
        rate = rospy.Rate(10)

        while self.current_ticks < target_ticks:
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 1.0  # Angular velocity (adjust as needed)
            self.pub.publish(self.cmd_msg)
            rate.sleep()

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
