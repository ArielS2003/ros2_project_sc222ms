# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import rclpy
import cv2
import numpy as np
import sys
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

        # Create a subscriber to the camera image topic (camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def callback(self, data):
        try:
            # Convert the ROS image message to OpenCV image format
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Create a window for displaying the image
            cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)

            # Display the image in the created window
            cv2.imshow('camera_feed', image)

            # Resize the window to fit the screen
            cv2.resizeWindow('camera_feed', 320, 240)

            # Wait for a key press for 3 ms, necessary for OpenCV window to update
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

# Function to handle cleanup when the program is interrupted
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    # Initialize the ROS node
    rclpy.init(args=None)
    colour_identifier = ColourIdentifier()

    # Ensure that the node keeps running
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(colour_identifier,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue

    except ROSInterruptException:
        pass

    # Close any OpenCV windows before shutting down
    cv2.destroyAllWindows()

# Check if the node is being run as the main program
if __name__ == '__main__':
    main()
