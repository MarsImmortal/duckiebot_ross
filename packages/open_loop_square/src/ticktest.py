#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.first_tick = None
        self.initial_ticks = None
        self.ticks_per_meter = None

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
            self.calibrate_ticks_per_meter()  # Calibrate ticks per meter
            self.move_straight(1.0)  # Move the robot forward by 1 meter

    def initial_tick(self, msg):
        self.first_tick = msg.data
        rospy.loginfo(f"first Ticks: {self.first_tick}")
        
    def encoder_callback(self, msg):
        # Store initial ticks value upon receiving first encoder message
        self.initial_ticks = msg.data
        rospy.loginfo(f"Initial Ticks: {self.initial_ticks}")

    def calibrate_ticks_per_meter(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.1  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {1} meters...")
        rospy.sleep(1 / 0.1)
        rospy.loginfo(f"Initial Ticks: {self.first_tick}")
        
        # Wait for the initial ticks to be set
        

    def move_straight(self, distance):
        # Convert desired distance to target ticks based on ticks_per_meter
        if self.ticks_per_meter is not None:
            target_ticks = int(distance * self.ticks_per_meter)

            # Set forward velocity
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.5  # Forward velocity (adjust as needed)
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Moving Forward by {distance} meters...")

            # Check if the robot has reached the target ticks
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown():
                current_ticks = rospy.get_param('/oryx/right_wheel_encoder_node/tick')  # Get current ticks
                if current_ticks >= target_ticks:
                    break
                rate.sleep()

            self.stop_robot()  # Stop the robot after reaching the desired distance
        else:
            rospy.logerr("Ticks per Meter is not calibrated.")

    def stop_robot(self):
        # Send zero velocities to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

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
