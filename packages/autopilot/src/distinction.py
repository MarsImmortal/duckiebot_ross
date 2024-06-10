#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from sensor_msgs.msg import Range

class Autopilot:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # Initialize variables for robot state and sensor data
        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.tof_distance = None
        self.tof_threshold = 0.5  # Threshold distance in meters for car following

        # Register the clean_shutdown function to be called on shutdown
        rospy.on_shutdown(self.clean_shutdown)
        
        # Initialize Publishers
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)

        # Initialize Subscribers
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.tof_callback, queue_size=1)
        
        ################################################################

        # Spin to keep the script running and handle callbacks
        rospy.spin() 

    # Callback function for AprilTag detections
    def tag_callback(self, msg):
        # Only process if the robot is in LANE_FOLLOWING state and we are not ignoring AprilTags
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        # Process the detected AprilTags
        self.move_robot(msg.detections)
    
    # ToF sensor callback
    def tof_callback(self, msg):
        # Update the ToF sensor distance
        self.tof_distance = msg.range
        # Check if the robot needs to overtake an object based on the ToF sensor data
        self.check_tof_distance()
 
    # Clean shutdown function to stop the robot safely
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Function to send zero velocity command to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # Zero linear velocity
        cmd_msg.omega = 0.0  # Zero angular velocity
        self.cmd_vel_pub.publish(cmd_msg)

    # Function to set the robot's state and publish it
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    # Function to check the ToF sensor distance and perform overtaking if needed
    def check_tof_distance(self):
        # Check if the ToF sensor data is available and the distance is less than the threshold
        if self.tof_distance is not None and self.tof_distance < self.tof_threshold:
            rospy.loginfo("Object detected in front. Stopping the robot...")
            # Change state to stop lane following
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            # Stop the robot
            self.stop_robot()
            rospy.sleep(3)  # Stop for 3 seconds

            rospy.loginfo("Overtaking the object...")
            # Perform overtaking maneuver
            self.overtake_object()

            # Change state back to lane following
            self.set_state("LANE_FOLLOWING")
            rospy.loginfo("Resuming lane following...")

    # Function to perform overtaking maneuver
    def overtake_object(self):
        # Example of simple overtaking maneuver
        # Move left to overtake
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.5
        cmd_msg.omega = 1.0  # Turn left
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Turn for 2 seconds

        # Move forward
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Move forward for 2 seconds

        # Move right to get back to lane
        cmd_msg.omega = -1.0  # Turn right
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Turn for 2 seconds

        # Move forward to ensure we are back in the lane
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(2)  # Move forward for 2 seconds

        # Stop the robot
        self.stop_robot()

    # Function to handle robot movement based on AprilTag detections
    def move_robot(self, detections):
        if len(detections) == 0:
            return

        # Example: Stop at Stop Sign
        for detection in detections:
            if detection.id == 1:  # Assuming tag ID 1 is the stop sign
                rospy.loginfo("Stop sign detected. Stopping the robot...")
                # Change state to stop lane following
                self.set_state("NORMAL_JOYSTICK_CONTROL")
                # Stop the robot
                self.stop_robot()
                rospy.sleep(3)  # Stop for 3 seconds

                # Move forward a bit to ensure the stop sign is out of view
                cmd_msg = Twist2DStamped()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.v = 0.5  # Move forward with a velocity of 0.5
                cmd_msg.omega = 0.0
                self.cmd_vel_pub.publish(cmd_msg)
                rospy.sleep(2)  # Move forward for 2 seconds

                # Stop the robot again
                self.stop_robot()

                # Ignore AprilTag messages for 5 seconds to avoid immediate re-triggering
                self.ignore_apriltag = True
                rospy.sleep(5)
                self.ignore_apriltag = False

                # Resume lane following
                self.set_state("LANE_FOLLOWING")
                rospy.loginfo("Resuming lane following...")

                return

# Main function
if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
