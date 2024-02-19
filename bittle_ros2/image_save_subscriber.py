import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        # Define the base directory to save images. Adjust as per your system's path.
        self.base_directory = os.path.join(os.path.expanduser('~'), 'Downloads')
        os.makedirs(self.base_directory, exist_ok=True)  # Ensure the directory exists

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        try:
            # Correctly convert your ROS CompressedImage message to OpenCV2
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        else:
            # Save your OpenCV2 image as a jpeg in the Downloads folder
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            image_path = os.path.join(self.base_directory, f'camera_image_{timestamp}.jpeg')
            cv2.imwrite(image_path, cv2_img)
            self.get_logger().info(f'Image saved to {image_path}')
            
            # Display the image stream
            cv2.imshow("Camera Image", cv2_img)
            cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    image_subscriber.destroy_node()
    cv2.destroyAllWindows()  # Make sure to close OpenCV windows
    rclpy.shutdown()

if __name__ == '__main__':
    main()


