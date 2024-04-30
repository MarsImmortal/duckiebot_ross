import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.obstacle_detected = False
        self.distance_traveled = 0.0
        self.encoder_ticks = 0
        self.target_ticks = 0

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            self.move_square()

    def range_callback(self, msg):
        if msg.range < 0.2:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def encoder_callback(self, msg):
        self.encoder_ticks = msg.data

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def move_forward(self, distance):
        self.target_ticks = self.encoder_ticks + distance_to_ticks(distance)

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.6
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")

        while not rospy.is_shutdown() and self.encoder_ticks < self.target_ticks:
            if self.obstacle_detected:
                self.stop_robot()
                break

            rospy.sleep(0.1)

        self.stop_robot()

    def turn_robot(self):
        self.target_ticks = self.encoder_ticks + angle_to_ticks(90)

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning...")

        while not rospy.is_shutdown() and self.encoder_ticks < self.target_ticks:
            rospy.sleep(0.1)

        self.stop_robot()

    def move_square(self):
        side_length = 0.5

        for _ in range(4):
            self.move_forward(side_length)
            self.turn_robot()

    def run(self):
        rospy.spin()

def distance_to_ticks(distance):
    # Calculate the number of encoder ticks required to move a specific distance
    # Calibration based on measured encoder ticks per meter
    ticks_per_meter = 135  # Example resolution
    return distance * ticks_per_meter

def angle_to_ticks(angle):
    # Calculate the number of encoder ticks required to rotate a specific angle
    # Calibration based on measured encoder ticks per degree of rotation
    ticks_per_degree = 10  # Example resolution
    return angle * ticks_per_degree

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
