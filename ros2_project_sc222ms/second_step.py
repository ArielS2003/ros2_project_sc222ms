# Exercise 2 - detecting two colours, and filtering out the third colour and background.
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

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define HSV ranges for Red, Green, Blue
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            hsv_red_lower1 = np.array([0, 100, 100])
            hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
            hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

            # Create masks
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
            red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

            # Combine all three
            combined_mask = cv2.bitwise_or(cv2.bitwise_or(green_mask, red_mask), blue_mask)
            final_image = cv2.bitwise_and(image, image, mask=combined_mask)

            # Show original and masks
            for name, mask in zip(['Green', 'Red', 'Blue'], [green_mask, red_mask, blue_mask]):
                cv2.namedWindow(f'{name}_mask', cv2.WINDOW_NORMAL)
                cv2.imshow(f'{name}_mask', mask)
                cv2.resizeWindow(f'{name}_mask', 320, 240)

            cv2.namedWindow('Combined_Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Combined_Detection', final_image)
            cv2.resizeWindow('Combined_Detection', 320, 240)
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