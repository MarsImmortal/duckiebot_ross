import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray, WheelEncoderStamped
from sensor_msgs.msg import Range  # Import Range message for ToF sensor

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_apriltag = False
        self.stop_sign_distance_threshold = 0.2  # Set your desired threshold distance here (in meters)
        self.object_detection_min = 0.3  # Minimum distance threshold for object detection
        self.object_detection_max = 0.5  # Maximum distance threshold for object detection

        # Initialize control variables
        self.prev_distance = None
        self.prev_ticks = None

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/oryx/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/oryx/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/oryx/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber('/oryx/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        rospy.Subscriber('/oryx/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

        rospy.spin()  # Spin forever but listen to message callbacks

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_apriltag:
            return
        
        rospy.loginfo("AprilTag detections received.")
        self.move_robot(msg.detections)
 
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        rospy.loginfo("Sending stop command to robot.")
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        rospy.loginfo(f"Setting state to {state}.")
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            rospy.loginfo("No AprilTags detected.")
            return
        
        for detection in detections:
            rospy.loginfo(f"Detected AprilTag with ID: {detection.tag_id}")
            if detection.tag_id == 32:  # Assuming tag ID 32 is the stop sign
                distance_to_tag = detection.transform.translation.z
                rospy.loginfo(f"Distance to AprilTag (ID 32): {distance_to_tag} meters")
                if distance_to_tag <= self.stop_sign_distance_threshold:
                    rospy.loginfo("Stop sign within threshold distance. Stopping the robot...")
                    
                    # Change state to stop lane following
                    self.set_state("NORMAL_JOYSTICK_CONTROL")
                    
                    # Stop the robot
                    self.stop_robot()
                    rospy.sleep(3)  # Stop for 3 seconds

                    # Move forward to clear the stop sign
                    cmd_msg = Twist2DStamped()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.v = 0.5  # Move forward with velocity 0.5
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

    def range_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        # Check if the distance is within the desired range for object detection
        if self.object_detection_min <= msg.range <= self.object_detection_max:
            # Object detected in front, stop the robot
            self.stop_robot()

            # Implement closed-loop movement based on distance readings
            if self.prev_distance is not None:
                # Compute control action based on the change in distance
                distance_change = msg.range - self.prev_distance
                velocity_change = distance_change * 0.5  # Adjust velocity based on distance change
                # Adjust robot's behavior based on distance change
                cmd_msg = Twist2DStamped()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.v = self.prev_velocity + velocity_change  # Adjust velocity
                cmd_msg.omega = 0.0
                self.cmd_vel_pub.publish(cmd_msg)
                self.prev_velocity = cmd_msg.v  # Update previous velocity
                
                # Move left for 1 second
                if distance_change < 0:
                    cmd_msg = Twist2DStamped()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.v = 0.0
                    cmd_msg.omega = 1.0  # Angular velocity for left turn
                    self.cmd_vel_pub.publish(cmd_msg)
                    rospy.sleep(2)  # Move left for 1 second

                # Move right for 1 second
                if distance_change > 0:
                    cmd_msg = Twist2DStamped()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.v = 0.0
                    cmd_msg.omega = -1.0  # Angular velocity for right turn
                    self.cmd_vel_pub.publish(cmd_msg)
                    rospy.sleep(2)  # Move right for 1 second

                # Move forward for 1 meter
                cmd_msg = Twist2DStamped()
                cmd_msg.header.stamp = rospy.Time.now()
                cmd_msg.v = 0.5  # Forward velocity
                cmd_msg.omega = 0.0
                self.cmd_vel_pub.publish(cmd_msg)
                rospy.sleep(2)  # Move forward for 2 seconds

            # Update previous distance for the next iteration
            self.prev_distance = msg.range
        else:
            # No obstacle detected within the specified range, resume lane following
            self.set_state("LANE_FOLLOWING")
            # Reset previous distance and velocity since the obstacle is no longer detected
            self.prev_distance = None
            self.prev_velocity = 0.0

    def encoder_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        # Update previous ticks for calculating distance traveled
        if self.prev_ticks is not None:
            ticks_change = msg.data - self.prev_ticks
            # Assuming each tick corresponds to 0.01 meters traveled
            distance_change = ticks_change * 0.01
            # Update previous distance
            self.prev_distance += distance_change

        # Update previous ticks for the next iteration
        self.prev_ticks = msg.data

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass