# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

import threading
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

        # Initialize CvBridge for converting ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Sensitivity for the color detection
        self.sensitivity = 10
        self.green_found = False
        self.min_area = 500  # Threshold for the minimum area

    def callback(self, data):
        try:
            # Convert the received image into an OpenCV image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Show the camera feed
            cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_feed', image)
            cv2.resizeWindow('camera_feed', 320, 240)
            cv2.waitKey(3)

            # Define the color range for green in HSV color space
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            # Convert the image from RGB to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Create a binary mask for the green areas
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # Find the contours of the green areas in the mask
            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Set a flag to check if green object is detected
            self.green_found = False

            if contours:
                # Loop over the contours and find the largest one
                largest_contour = max(contours, key=cv2.contourArea)

                # Calculate the area of the largest contour
                contour_area = cv2.contourArea(largest_contour)
                print(f"Contour Area: {contour_area}")

                # Check if the contour area is larger than the threshold
                if contour_area > self.min_area:
                    # Draw a circle around the largest contour
                    (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    # Set the green_found flag to True
                    self.green_found = True

            # If green object is detected and its area is above the threshold
            if self.green_found:
                print("Green object detected and is above the size threshold!")

            # Show the result image with detected green area
            cv2.namedWindow('result_feed', cv2.WINDOW_NORMAL)
            cv2.imshow('result_feed', image)
            cv2.resizeWindow('result_feed', 320, 240)
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
