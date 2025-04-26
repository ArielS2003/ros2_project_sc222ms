# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal

class FirstStep(Node):
    def __init__(self):
        super().__init__('first_step')

        # Initialize CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Create a subscriber to the camera image topic (camera/image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Subscribe to the topic
            self.callback,         # Callback function to handle image data
            10                     # Queue size
        )
        self.subscription  # Prevent unused variable warning

    def callback(self, data):
        try:
            # Convert the ROS image message to OpenCV image format ('bgr8' represents BGR format)
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Create a resizable window to display the image
            cv2.namedWindow('camera_feed', cv2.WINDOW_NORMAL)

            # Display the image
            cv2.imshow('camera_feed', image)

            # Resize the window to fit the screen
            cv2.resizeWindow('camera_feed', 320, 240)

            # Wait for 3 milliseconds to allow the window to update
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

def main():
    # Initialize the ROS2 node
    rclpy.init(args=None)
    first_step = FirstStep()

    # Signal handler to gracefully shutdown when Ctrl+C is pressed
    def signal_handler(sig, frame):
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Close any OpenCV windows
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spin the ROS2 node to handle incoming messages
        rclpy.spin(first_step)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure OpenCV windows are closed before shutting down
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
