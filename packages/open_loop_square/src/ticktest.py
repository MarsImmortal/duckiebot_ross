#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.encoder_ticks_start = 0
        self.encoder_ticks_end = 0
        self.ticks_per_meter = 135  # Example resolution (adjust based on your system)
        self.target_distance = 1.0  # Target distance to move (in meters)

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            self.move_forward()

    def encoder_callback(self, msg):
        # Update encoder ticks
        self.encoder_ticks_end = msg.data
        rospy.loginfo(f"Encoder ticks: {self.encoder_ticks_end}")

    def move_forward(self):
        # Store initial encoder ticks before movement
        self.encoder_ticks_start = self.encoder_ticks_end

        # Calculate target ticks for the target distance
        target_ticks = self.encoder_ticks_start + self.distance_to_ticks(self.target_distance)

        # Send velocity command to move forward
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.6  # Adjust linear velocity as needed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving forward by {self.target_distance} meters...")

        # Monitor encoder ticks until reaching the target ticks (target distance)
        while not rospy.is_shutdown() and self.encoder_ticks_end < target_ticks:
            rospy.sleep(0.1)

        # Stop the robot after reaching the target distance
        self.stop_robot()

    def stop_robot(self):
        # Send zero velocity command to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def distance_to_ticks(self, distance):
        # Calculate the number of encoder ticks required to move a specific distance
        return distance * self.ticks_per_meter

    def run(self):
        rospy.spin()  # Keeps the node from exiting until shutdown

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
