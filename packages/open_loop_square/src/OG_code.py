#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)

        # Check the FSM state and perform actions accordingly
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()  # Stop the robot if in joystick control mode
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a second for the node to be ready
            self.move_square()  # Move the robot in a square pattern

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    # Move the robot in a square pattern
    def move_square(self):
        # Define the side length of the square (adjust as needed)
        side_length = 0.1  # meters

        # Move the robot forward and then turn 90 degrees four times to form a square
        for _ in range(4):
            # Move forward for the specified side length
            self.move_forward(side_length)
            # Turn the robot 90 degrees (adjust the angular velocity for the turn)
            self.turn_robot()

        # Stop the robot after completing the square
        self.stop_robot()

    # Move the robot forward by a specified distance (side_length)
    def move_forward(self, distance):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.6  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")
        rospy.sleep(distance / 0.1)
        #self.stop_robot()  # Adjust sleep time based on the robot's speed and required distance

    def turn_robot(self):
        # Set the angular velocity to turn 90 degrees (adjust as needed)
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 1  # Adjust the angular velocity for a slower turn
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning...")
        rospy.sleep(0.75)
        #self.stop_robot()  # Adjust sleep time for a 90-degree turn based on the turning speed


    # Run the ROS node (spin forever)
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