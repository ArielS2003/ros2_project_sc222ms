# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        # Initialize CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Sensitivity for color detection and flags for detected colors
        self.sensitivity = 10
        self.green_found = False
        self.blue_found = False
        self.too_close = False
        
        # Publisher to send movement commands to the robot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

    def callback(self, data):
        # Convert the received image to OpenCV format
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Display the camera feed (optional)
        cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('camera_feed', image)
        cv2.resizeWindow('camera_feed', 320, 240)
        cv2.waitKey(3)

        # Define the color ranges for green and blue
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([130 + self.sensitivity, 255, 255])

        # Convert the image from BGR to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks for green and blue objects
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        # Find contours of green and blue objects
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if green object is detected
        self.green_found = False
        if len(green_contours) > 0:
            # Find the largest green contour
            green_contour = max(green_contours, key=cv2.contourArea)

            # Calculate the area of the green contour
            if cv2.contourArea(green_contour) > 100:  # Threshold area
                self.green_found = True
                # Draw a circle around the green object
                (x, y), radius = cv2.minEnclosingCircle(green_contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (0, 255, 0), 2)

                # Check if the object is too close or too far
                if cv2.contourArea(green_contour) > 30000:
                    self.too_close = True
                else:
                    self.too_close = False

        # Check if blue object is detected (stop the robot if blue is detected)
        self.blue_found = False
        if len(blue_contours) > 0:
            self.blue_found = True
            print("Blue object detected! Stopping the robot.")
            self.stop()

        # Publish movement commands based on the detected colors
        if self.green_found and not self.blue_found:
            if self.too_close:
                self.walk_backward()
            else:
                self.walk_forward()
        elif not self.green_found:
            self.stop()

        # Display the processed image with contours
        cv2.namedWindow('processed_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('processed_feed', image)
        cv2.resizeWindow('processed_feed', 320, 240)
        cv2.waitKey(3)

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Move forward with 0.2 m/s
        self.publisher.publish(desired_velocity)

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2  # Move backward with 0.2 m/s
        self.publisher.publish(desired_velocity)

    def stop(self):
        desired_velocity = Twist()
        self.publisher.publish(desired_velocity)  # Send zero velocity to stop the robot

# Main function to run the node and handle exceptions
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    # Initialize the ROS2 node
    rclpy.init(args=None)
    robot = Robot()

    # Signal handling to shut down ROS2 node gracefully
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            pass
    except ROSInterruptException:
        pass

    # Close all OpenCV windows when exiting
    cv2.destroyAllWindows()

# Check if the node is being executed as the main program
if __name__ == '__main__':
    main()
