import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from sensor_msgs.msg import Range

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.obstacle_threshold = 0.3
        self.obstacle_detected = False
        self.square_side_length = 0.5

        rospy.init_node('drive_square_node', anonymous=True)
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.loginfo("Executing Lane Following Mode...")
            if not self.obstacle_detected:
                self.move_square()

    def range_callback(self, msg):
        if msg.range >= 0.1 and msg.range <= 1.0:  # Obstacle detection range
            if msg.range < self.obstacle_threshold:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def move_forward(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.3  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def move_square(self):
        rospy.loginfo("Moving forward...")
        while not self.obstacle_detected:
            self.move_forward()
            rospy.sleep(0.1)  # Check every 0.1 seconds if obstacle is detected
        self.stop_robot()
        rospy.loginfo("Obstacle detected, waiting for removal...")

        while self.obstacle_detected:
            rospy.sleep(0.1)  # Check every 0.1 seconds if obstacle is removed
        rospy.loginfo("Obstacle removed, starting square movement...")

        # Make a square of length 0.5
        for _ in range(4):
            self.move_forward()
            rospy.sleep(self.square_side_length / 0.3)  # Adjust sleep time based on the robot's speed and required distance
            self.stop_robot()
            rospy.sleep(0.5)  # Wait for a moment before rotating
            self.rotate(90)

    def rotate(self, angle):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.3  # Angular velocity (adjust as needed)
        self.pub.publish(self.cmd_msg)
        rospy.sleep(angle * 0.01)  # Adjust sleep time based on the required rotation angle
        self.stop_robot()

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
