import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.detection_publisher = self.create_publisher(
            DetectionResults,  # This is a hypothetical message type; you may need to define it based on your requirements
            '/detection_results',
            10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/reid/autonomous-bittle/vision-based-command/acorns_and_decahedrons_v1/best.pt')
        self.model.conf = 0.5  # Confidence threshold

    def listener_callback(self, data):
        frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        results = self.model([frame])
        # Process results and publish detection information
        # You would convert your detection results into a ROS message here and publish it

def main(args=None):
    rclpy.init(args=args)
    detection_node = ObjectDetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
