import rclpy
from rclpy.node import Node
from bittle_msgs.srv import CapturePhoto
from cv_bridge import CvBridge
import cv2

class PhotoClient(Node):

    def __init__(self):
        super().__init__('photo_client_node')
        self.client = self.create_client(CapturePhoto, 'capture_photo')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = CapturePhoto.Request()
        self.bridge = CvBridge()
        self.send_request()

    def send_request(self):
        future = self.client.call_async(self.request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received image')
            image = self.bridge.imgmsg_to_cv2(response.image, "bgr8")
            cv2.imshow("Captured Image", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    photo_client = PhotoClient()
    rclpy.spin(photo_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

