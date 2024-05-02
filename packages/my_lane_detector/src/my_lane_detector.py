def image_callback(self, msg):
    rospy.loginfo("Image received")

    # Convert ROS image message to OpenCV image
    img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    if img is None:
        rospy.logwarn("Received empty image. Skipping processing.")
        return

    # Crop the image to focus on the road region (adjust crop coordinates as needed)
    height, width, _ = img.shape
    roi_top = int(height * 0.8)
    roi_bottom = height
    roi_left = int(width * 0.2)
    roi_right = int(width * 0.8)

    # Check cropping bounds
    rospy.loginfo(f"Cropping bounds: top={roi_top}, bottom={roi_bottom}, left={roi_left}, right={roi_right}")

    # Verify valid cropping region
    if roi_top < roi_bottom and roi_left < roi_right:
        cropped_img = img[roi_top:roi_bottom, roi_left:roi_right]

        if cropped_img is not None:
            # Convert cropped image to HSV color space
            hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

            # Further processing (e.g., color masking, edge detection)
            # ...

            # Display images for debugging
            cv2.imshow('Cropped Image', cropped_img)
            cv2.imshow('HSV Image', hsv_img)
            cv2.waitKey(1)
        else:
            rospy.logwarn("Empty cropped image. Skipping further processing.")
    else:
        rospy.logwarn("Invalid cropping bounds. Skipping processing.")

def run(self):
    rospy.spin()  # Spin forever but listen to message callbacks
