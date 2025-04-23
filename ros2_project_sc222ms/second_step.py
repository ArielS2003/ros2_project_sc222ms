# Exercise 2 - detecting two colours, and filtering out the third colour and background.

import threading
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # Initialize CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Create a subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Sensitivity value for color detection
        self.sensitivity = 10

    def callback(self, data):
        try:
            # Convert the received image into an OpenCV image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Create a window to show the original image
            cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_feed', image)
            cv2.resizeWindow('camera_feed', 320, 240)
            cv2.waitKey(3)

            # Set the upper and lower bounds for the colours you wish to detect
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
            hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])

            # Convert the image from RGB to HSV color space
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Filter out everything but particular colours using cv2.inRange()
            green_image = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            red_image = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)

            # Combine the masks for green and red using bitwise OR
            combined_mask = cv2.bitwise_or(green_image, red_image)

            # Apply the mask to the original image
            final_image = cv2.bitwise_and(image, image, mask=combined_mask)

            # Show the processed green image
            cv2.namedWindow('green_feed', cv2.WINDOW_NORMAL)
            cv2.imshow('green_feed', green_image)
            cv2.resizeWindow('green_feed', 320, 240)
            cv2.waitKey(3)

            # Show the final image with both green and red detected
            cv2.namedWindow('final_feed', cv2.WINDOW_NORMAL)
            cv2.imshow('final_feed', final_image)
            cv2.resizeWindow('final_feed', 320, 240)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")


# Main function to run the node and handle exceptions
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    # Initialize the ROS2 node
    rclpy.init(args=None)
    colour_identifier = ColourIdentifier()

    # Signal handling to shut down ROS2 node gracefully
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(colour_identifier,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Close all OpenCV windows when exiting
    cv2.destroyAllWindows()


# Check if the node is being executed as the main program
if __name__ == '__main__':
    main()
