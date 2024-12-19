import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
from bittle_msgs.msg import AprilTag

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            1)
        self.image_publisher = self.create_publisher(CompressedImage, 'apriltag_detections/compressed', 10)
        self.detection_publisher = self.create_publisher(AprilTag, '/apriltag_topic', 10)
        self.bridge = CvBridge()

        self.camera_matrix = np.array([[1.58644533e+03, 0.00000000e+00, 1.09010209e+03],
                                       [0.00000000e+00, 1.58999207e+03, 5.54331347e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype="double")
        self.dist_coeffs = np.array([[6.67373874e-01, -3.70098568e+00, 5.15550204e-03, 5.80996850e-02,
                                      5.60129213e+00]], dtype="double")
        self.tag_size = 0.1  # Example: 10 cm
        self.detector = apriltag.Detector()

    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray_frame)
        center_list = []
        rotation_list = []
        tag_id_list = []

        for i, r in enumerate(results):
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            object_points = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [self.tag_size/2, -self.tag_size/2, 0],
                [self.tag_size/2, self.tag_size/2, 0],
                [-self.tag_size/2, self.tag_size/2, 0]
            ], dtype="double")

            image_points = np.array([r.corners[0], r.corners[1], r.corners[2], r.corners[3]], dtype="double")
            _, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
            position = tvec.flatten()
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            roll = np.arctan2(rotation_matrix[2][1], rotation_matrix[2][2])
            pitch = np.arctan2(-rotation_matrix[2][0], np.sqrt(rotation_matrix[2][1]**2 + rotation_matrix[2][2]**2))
            yaw = np.arctan2(rotation_matrix[1][0], rotation_matrix[0][0])

            cx = results[i].center[0]
            cy = results[i].center[1]
            #self.get_logger().info(f"AprilTag detected at: X={cx:.0f}, Y={cy:.0f} tag id: {results[i].tag_id}")
            #self.get_logger().info(f'Tag orientation: {np.degrees(yaw):.0f} tag id: {results[i].tag_id}')
            tag_id_list.append(results[i].tag_id)
            center_list.extend([cx, cy])
            rotation_list.extend([roll, pitch, yaw])


        image_message = self.bridge.cv2_to_compressed_imgmsg(frame)
        april_tag_message = AprilTag()
        april_tag_message.tag_id = tag_id_list
        april_tag_message.center = center_list
        april_tag_message.orientation = rotation_list
        self.detection_publisher.publish(april_tag_message)
        self.image_publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
