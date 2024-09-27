# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from cv_bridge import CvBridge
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# def find_and_match_features(img1, img2):
#     # Initialize ORB detector
#     orb = cv2.ORB_create()

#     # Find the keypoints and descriptors with ORB
#     kp1, des1 = orb.detectAndCompute(img1, None)
#     kp2, des2 = orb.detectAndCompute(img2, None)

#     # Create BFMatcher object
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#     # Match descriptors
#     matches = bf.match(des1, des2)

#     # Sort them in the order of their distance
#     matches = sorted(matches, key=lambda x: x.distance)

#     # Extract location of good matches
#     points1 = np.zeros((len(matches), 2), dtype=np.float32)
#     points2 = np.zeros((len(matches), 2), dtype=np.float32)

#     for i, match in enumerate(matches):
#         points1[i, :] = kp1[match.queryIdx].pt
#         points2[i, :] = kp2[match.trainIdx].pt

#     return points1, points2, kp1, kp2, matches

# def triangulate_points(P1, P2, points1, points2):
#     # Triangulate points
#     points4D = cv2.triangulatePoints(P1, P2, points1.T, points2.T)
#     # Convert from homogeneous to Cartesian coordinates
#     points3D = points4D[:3] / points4D[3]
#     return points3D
# class CameraSubscriber(Node):
#     def __init__(self):
#         super().__init__('camera_subscriber')
#         self.bridge = CvBridge()
#         self.initialize_camera_matrices()

#         # Subscriptions
#         self.create_subscription(Image, '/stereo_left', self.left_image_callback, 10)
#         self.create_subscription(Image, '/stereo_right', self.right_image_callback, 10)

#         # Image storage
#         self.left_image = None
#         self.right_image = None

#     def initialize_camera_matrices(self):
#         fx = 1143.072833 
#         cx = 959.967467
#         fy = 1143.126174
#         cy = 719.840299
#         tx = -411.87  # Assuming the right camera is to the left of the left camera

#         self.P1 = np.array([[fx, 0, cx, 0], [0, fy, cy, 0], [0, 0, 1, 0]])
#         self.P2 = np.array([[fx, 0, cx, tx], [0, fy, cy, 0], [0, 0, 1, 0]])

#     def left_image_callback(self, msg):
#         self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         self.try_process_images()

#     def right_image_callback(self, msg):
#         self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         self.try_process_images()

#     def try_process_images(self):
#         if self.left_image is not None and self.right_image is not None:
#             points1, points2, kp1, kp2, matches = find_and_match_features(self.left_image, self.right_image)
#             points3D = triangulate_points(self.P1, self.P2, points1, points2)

#             # Display and annotate
#             self.display_and_annotate(self.left_image, self.right_image, points1, points2, points3D)

#             self.left_image = None
#             self.right_image = None

#     def display_and_annotate(self, img1, img2, points1, points2, points3D):
#         # Combine images horizontally
#         h1, w1, _ = img1.shape
#         h2, w2, _ = img2.shape
#         height = max(h1, h2)
#         width = w1 + w2
#         img_combined = np.zeros((height, width, 3), dtype=np.uint8)
#         img_combined[:h1, :w1, :] = img1
#         img_combined[:h2, w1:w1+w2, :] = img2

#         # Annotate points with their depth
#         font = cv2.FONT_HERSHEY_SIMPLEX
    
#         for pt1, pt2, pt3D in zip(points1, points2, points3D.T):
    
#             pt1 = (int(pt1[0]), int(pt1[1]))
#             pt2 = (int(pt2[0]) + w1, int(pt2[1]))  # shift x coordinate for right image
#             cv2.circle(img_combined, pt1, 5, (0, 255, 0), -1)
#             cv2.circle(img_combined, pt2, 5, (0, 0, 255), -1)
#             distance = np.linalg.norm(pt3D)
#             cv2.putText(img_combined, f"{distance:.2f}", pt1, font, 0.5, (0, 0, 255), 2)

#         cv2.imshow("Stereo Matches with Distance", img_combined)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def find_and_match_features(img1, img2):
    # Initialize ORB detector
    orb = cv2.ORB_create()

    # Find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors
    matches = bf.match(des1, des2)

    # Sort them in the order of their distance
    matches = sorted(matches, key=lambda x: x.distance)

    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points1[i, :] = kp1[match.queryIdx].pt
        points2[i, :] = kp2[match.trainIdx].pt

    return points1, points2, kp1, kp2, matches

def triangulate_points(P1, P2, points1, points2):
    # Triangulate points
    points4D = cv2.triangulatePoints(P1, P2, points1.T, points2.T)
    # Convert from homogeneous to Cartesian coordinates
    points3D = points4D[:3] / points4D[3]
    return points3D

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()

        # Initialize matplotlib figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()

        # Camera projection matrices
        fx = 1143.072833 
        cx = 959.967467
        fy = 1143.126174
        cy = 719.840299
        tx = -686.45  # Negative if the second camera is to the left of the first camera
        self.P1 = np.array([[fx, 0, cx, 0], [0, fy, cy, 0], [0, 0, 1, 0]])
        self.P2 = np.array([[fx, 0, cx, tx], [0, fy, cy, 0], [0, 0, 1, 0]])

        # Subscriptions
        self.create_subscription(Image, '/stereo_left', self.left_image_callback, 10)
        self.create_subscription(Image, '/stereo_right', self.right_image_callback, 10)

        # Image storage
        self.left_image = None
        self.right_image = None

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process_images()

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process_images()

    def try_process_images(self):
        if self.left_image is not None and self.right_image is not None:
            points1, points2, kp1, kp2, matches = find_and_match_features(self.left_image, self.right_image)
            points3D = triangulate_points(self.P1, self.P2, points1, points2)
            print(points3D)
            self.update_plot(points3D)
            self.display_feature_matches(self.left_image, self.right_image, kp1, kp2, matches)
            self.left_image = None
            self.right_image = None

    def update_plot(self, points3D):
        self.ax.clear()
        self.ax.scatter(points3D[0], points3D[1], points3D[2], color='green')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.draw()
        plt.pause(0.001)

    def display_feature_matches(self, img1, img2, kp1, kp2, matches):
        # Draw matches and keypoints on the images
        matched_img = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("Feature Matches", matched_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()