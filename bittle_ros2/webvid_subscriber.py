import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StreamSubscriber(Node):
    def __init__(self):
        super().__init__('stream_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/stream', self.listener_callback, 30)
        cv2.namedWindow("MJPEG Stream", cv2.WINDOW_NORMAL)

    def listener_callback(self, data):
        try:
            # Convert the incoming ROS Image message to an OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            # Display the frame
            cv2.imshow("MJPEG Stream", current_frame)
            # Close the window if 'q' is pressed
            if cv2.waitKey(1) == ord('q'):
                self.get_logger().info("Stream closed by user.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StreamSubscriber()
    executor = SingleThreadedExecutor()  # Use single-threaded executor
    executor.add_node(node)

    try:
        executor.spin()  # Spin using the executor
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

