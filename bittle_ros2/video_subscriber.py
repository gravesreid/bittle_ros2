import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Correct message type for compressed images
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from rclpy.executors import MultiThreadedExecutor

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        
        # Create the subscriber. This subscriber will receive a CompressedImage
        # from the /image_raw/compressed topic. The queue size is 10 messages.
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
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    rclpy.spin(image_subscriber, executor=executor)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()