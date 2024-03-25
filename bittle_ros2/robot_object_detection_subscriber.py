#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  
from bittle_msgs.msg import Detection

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            1)
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/root/best.pt')  # Update the path to your model
        self.model.conf = 0.5  # Set confidence threshold
        
        # Create detection publisher
        self.detection_publisher = DetectionPublisher()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Perform object detection
        results = self.model([current_frame])  
        

        
        # Publish detection results
        if len(results) > 0:
            result_list = (results[0].boxes.cls).cpu().tolist()
            xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
        
        detection_info = [{'results': result_list, 'xywhn_list': xywhn_list}]
        
        self.detection_publisher.publish_detection_info(detection_info)
        
class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher = self.create_publisher(Detection, 'detection_topic', 10)
        
    def publish_detection_info(self, detection):
        msg = Detection()
        msg.results = [int(result) for result in detection[0]['results']]
        print('results:', msg.results)
        # Initialize an empty list to hold the flattened and converted values
        flattened_xywhn_list = []

        # Iterate through each sublist in 'xywhn_list'
        for sublist in detection[0]['xywhn_list']:
            # Check if the item is a list (to handle nested lists)
            if isinstance(sublist, list):
                # Iterate through each item in the sublist
                for item in sublist:
                    # Convert each item to float and append to the flattened list
                    flattened_xywhn_list.append(float(item))
            else:
                # If the item is not a list, convert and append directly
                flattened_xywhn_list.append(float(sublist))

        # Assign the flattened list of floats to 'msg.xywhn_list'
        msg.xywhn_list = flattened_xywhn_list
        print('xywhn_list:', msg.xywhn_list)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing detection')


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
