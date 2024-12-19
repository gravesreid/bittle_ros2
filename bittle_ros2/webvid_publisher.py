import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MJPEGStreamPublisher(Node):
    def __init__(self):
        super().__init__('mjpeg_stream_publisher')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/stream', 30)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Adjust as needed
        self.stream_url = "http://192.168.1.187:8000/stream.mjpg"
        self.cap = cv2.VideoCapture(self.stream_url)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open MJPEG stream.")
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame from MJPEG stream.")
            return

        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info("Published frame from MJPEG stream.")

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = MJPEGStreamPublisher()
    executor = SingleThreadedExecutor()  # Use single-threaded executor
    executor.add_node(node)
    
    try:
        executor.spin()  # Spin using the executor
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

