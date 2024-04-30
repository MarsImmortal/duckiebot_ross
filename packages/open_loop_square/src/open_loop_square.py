#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.obstacle_detected = False
        self.encoder_ticks = 0
        self.ticks_per_meter = 534  # Ticks per meter for your robot
        self.side_length = 0.5  # Adjust side length as needed for your square

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
        # Check for obstacles within a defined range (e.g., 0.2 meters)
        if msg.range < 0.05:
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
        target_ticks = self.encoder_ticks + distance * self.ticks_per_meter

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.6
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")

        while not rospy.is_shutdown() and self.encoder_ticks < target_ticks:
            if self.obstacle_detected:
                # Avoid obstacle by turning right
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.0
                self.cmd_msg.omega = -1  # Turn right (adjust angular velocity as needed)
                self.pub.publish(self.cmd_msg)
                rospy.sleep(1)  # Turn for 1 second

                # Resume forward movement
                self.cmd_msg.header.stamp = rospy.Time.now()
                self.cmd_msg.v = 0.1
                self.cmd_msg.omega = 0.0
                self.pub.publish(self.cmd_msg)
                rospy.loginfo("Resuming forward movement after avoiding obstacle...")

            rospy.sleep(0.1)

        self.stop_robot()

    def turn_robot(self):
        target_ticks = self.encoder_ticks + 90  # Rotate 90 degrees

        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 1
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning...")

        while not rospy.is_shutdown() and self.encoder_ticks < target_ticks:
            rospy.sleep(0.1)

        self.stop_robot()

    def move_square(self):
        for _ in range(4):
            self.move_forward(self.side_length)
            self.turn_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
