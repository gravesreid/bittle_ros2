import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Correct message type for compressed images
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from rclpy.executors import MultiThreadedExecutor
import os
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        #make a counter to save every 5th frame
        self.frame_counter = 0
        self.base_directory = "/home/reid/Downloads"
        os.makedirs(self.base_directory, exist_ok=True)
        # Create the subscriber. This subscriber will receive a CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.listener_callback, 
            1)
        self.subscription  # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS CompressedImage message to OpenCV image
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        
        # Display image
        cv2.imshow("camera", current_frame)
        
        # save every 5th frame
        if self.frame_counter % 5 == 0:
            filename = os.path.join(self.base_directory, f'frame_{self.frame_counter}.jpg')
            #threading.Thread(target=self.save_image, args=(current_frame, filename)).start()
        self.frame_counter += 1
        
        cv2.waitKey(1)
    def save_image(self, frame, filename):
        cv2.imwrite(filename, frame)
        self.get_logger().info(f'Image saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    rclpy.spin(image_subscriber, executor=executor)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()