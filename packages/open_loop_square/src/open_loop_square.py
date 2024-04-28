#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Range  # Import Range message type for ToF sensor

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.obstacle_detected = False  # Flag to indicate obstacle detection
        self.distance_traveled = 0.0     # Track the distance traveled

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/oryx/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

    # Callback function to handle FSM state changes
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)

        # Check the FSM state and perform actions accordingly
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()  # Stop the robot if in joystick control mode
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a second for the node to be ready
            self.move_square()  # Move the robot in a square pattern

    # Callback function for ToF range messages
    def range_callback(self, msg):
        # Check the distance reading from the ToF sensor
        if msg.range < 0.2:  # Adjust the threshold as needed (e.g., 0.2 meters)
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

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
        side_length = 0.5  # meters

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
        self.cmd_msg.v = 0.1  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving Forward by {distance} meters...")

        # Monitor distance traveled to handle obstacles
        initial_distance = self.distance_traveled
        target_distance = initial_distance + distance

        while not rospy.is_shutdown() and self.distance_traveled < target_distance:
            rospy.sleep(0.1)  # Check distance traveled every 0.1 seconds

        # Stop the robot after reaching the desired distance
        self.stop_robot()

    def turn_robot(self):
        # Set the angular velocity to turn 90 degrees (adjust as needed)
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.5  # Adjust the angular velocity for a slower turn
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turning...")

        # Monitor angle turned to complete the turn (adjust based on turning performance)
        initial_angle = self.current_angle  # Replace with actual angle measurement
        target_angle = initial_angle + 1.57  # Target angle for 90 degrees (adjust as needed)

        while not rospy.is_shutdown() and self.current_angle < target_angle:
            rospy.sleep(0.1)  # Check angle turned every 0.1 seconds

        # Stop the robot after completing the turn
        self.stop_robot()

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
