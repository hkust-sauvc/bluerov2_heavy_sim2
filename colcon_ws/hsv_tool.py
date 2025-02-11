#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def nothing(x):
    """Dummy callback for trackbar (required by OpenCV)."""
    pass

class HSVFinder(Node):
    def __init__(self):
        super().__init__('hsv_finder_node')
        
        # Create OpenCV windows
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

        # Create trackbars for lower and upper HSV values
        # Hue range in OpenCV: 0-179, Saturation & Value range: 0-255
        cv2.createTrackbar("H_min", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("H_max", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("S_min", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("S_max", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("V_min", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("V_max", "Trackbars", 255, 255, nothing)

        # Create a CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/left/image_rect',  # <-- change if your topic name is different
            self.image_callback,
            10
        )

        self.get_logger().info("HSV Finder Node has been started.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get trackbar positions
        h_min = cv2.getTrackbarPos("H_min", "Trackbars")
        h_max = cv2.getTrackbarPos("H_max", "Trackbars")
        s_min = cv2.getTrackbarPos("S_min", "Trackbars")
        s_max = cv2.getTrackbarPos("S_max", "Trackbars")
        v_min = cv2.getTrackbarPos("V_min", "Trackbars")
        v_max = cv2.getTrackbarPos("V_max", "Trackbars")

        # Convert image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper HSV range
        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Create a mask based on the trackbar HSV range
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        # Show the original image and the mask
        cv2.imshow("Original", cv_image)
        cv2.imshow("Mask", mask)

        cv2.waitKey(1)  # Important for GUI events

def main(args=None):
    rclpy.init(args=args)
    node = HSVFinder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
