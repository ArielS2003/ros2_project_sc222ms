# Exercise 3 - If blue object is detected, and above a certain size, then send a message (print or use lab2)

import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal

class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.sensitivity = 10
        self.blue_found = False
        self.min_area = 1000

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define HSV range for blue
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            self.blue_found = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area > self.min_area:
                    (x, y), radius = cv2.minEnclosingCircle(largest)
                    cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 0), 2)
                    self.blue_found = True
                    print("Blue object detected, area:", area)

            cv2.imshow('blue_mask', blue_mask)
            cv2.imshow('detection_result', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()
        cv2.destroyAllWindows()

    rclpy.init()
    node = ColourIdentifier()
    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
