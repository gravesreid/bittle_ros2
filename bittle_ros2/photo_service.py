import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bittle_msgs.srv import CapturePhoto
import cv2
from cv_bridge import CvBridge

class PhotoService(Node):

    def __init__(self):
        super().__init__('photo_service_node')
        self.srv = self.create_service(CapturePhoto, 'capture_photo', self.capture_photo_callback)
        self.bridge = CvBridge()

    def capture_photo_callback(self, request, response):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            response.image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        else:
            self.get_logger().error('Failed to capture image')
        cap.release()
        return response

def main(args=None):
    rclpy.init(args=args)
    photo_service = PhotoService()
    rclpy.spin(photo_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()