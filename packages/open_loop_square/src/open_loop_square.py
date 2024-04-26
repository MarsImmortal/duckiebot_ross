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
        
    # Callback function to handle FSM state changes
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        
        # Check the FSM state and perform actions accordingly
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()  # Stop the robot if in joystick control mode
        elif msg.state == "Custom_State":
            rospy.sleep(1)  # Wait for a second for the node to be ready
            self.move_robot()  # Move the robot forward and backward

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    # Move the robot forward and then backward
    def move_robot(self):
        # Move forward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # Forward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving Forward...")
        rospy.sleep(2.0)  # Adjust sleep time based on the desired distance (1 meter at 0.5 m/s)

        # Stop the robot before reversing
        self.stop_robot()

        # Move backward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -0.5  # Backward velocity (adjust as needed)
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Moving Backward...")
        rospy.sleep(2.0)  # Adjust sleep time based on the desired distance (1 meter at 0.5 m/s)

        # Stop the robot after completing the movement
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
