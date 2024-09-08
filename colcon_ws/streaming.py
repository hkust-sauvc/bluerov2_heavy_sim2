import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()

        # Subscription for the left camera
        self.left_camera_subscription = self.create_subscription(
            Image,
            '/stereo_left/simulated_image',
            self.left_image_callback,
            10)

        # Subscription for the right camera
        self.right_camera_subscription = self.create_subscription(
            Image,
            '/stereo_right/simulated_image',
            self.right_image_callback,
            10)

        # Variables to hold the latest images
        self.left_image = None
        self.right_image = None

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.try_stitch_images()

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.try_stitch_images()

    def try_stitch_images(self):
        if self.left_image is not None and self.right_image is not None:
            # Stitch images
            stitched_image = self.stitch_images(self.left_image, self.right_image)
            cv2.imshow("Cameras View", stitched_image)
            # Check if a specific key is pressed to close the window (e.g., 'q' or ESC)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_node()

    def stitch_images(self, left_img, right_img):
        # Simple horizontal concatenation for stitching
        stitched_image = cv2.hconcat([left_img, right_img])
        return stitched_image

    def stop_node(self):
        # Destroy the OpenCV window
        cv2.destroyAllWindows()
        # Shutdown the ROS node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    # Ensure that if ROS is down, OpenCV window also gets closed
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
