import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import hashlib

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        try:
            # Convert your ROS CompressedImage message to OpenCV2
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        else:
            # Calculate and print the hash sum of the image pixels
            # Convert the image to bytes
            img_bytes = cv2_img.tobytes()
            # Create a hash object
            hash_obj = hashlib.sha256()
            # Update the hash object with the bytes of the image
            hash_obj.update(img_bytes)
            # Get the hexadecimal digest of the hash
            hash_hex = hash_obj.hexdigest()
            self.get_logger().info(f'Hash sum of the image pixels: {hash_hex}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
