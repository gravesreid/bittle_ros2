import rclpy
from rclpy.node import Node
from bittle_msgs.msg import Yolo, AprilTag
from sensor_msgs.msg import CompressedImage
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import cv2
import io

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.yolo_subscription = self.create_subscription(
            Yolo,
            '/yolo_topic',
            self.yolo_callback,
            10)
        self.apriltag_subscription = self.create_subscription(
            AprilTag,
            '/apriltag_topic',
            self.apriltag_callback,
            10)
        self.map_publisher = self.create_publisher(
            CompressedImage, 
            '/map_image/compressed',
              10)
        self.yolo_positions = []
        self.yolo_boxes = []
        self.apriltag_positions = []
        self.bridge = CvBridge()

    def yolo_callback(self, msg):
        self.yolo_positions.clear()
        yolo_list = msg.xywh_list
        # List contains all detection in order x, y, w, h, need to parse
        for i in range(len(yolo_list)//4):
            x,y,w,h = yolo_list[i*4:i*4+4]
            self.yolo_positions.append((x, y))
            self.yolo_boxes.append((x, y, w, h))
        self.update_map()

    def apriltag_callback(self, msg):
        self.apriltag_positions.clear()
        x, y = msg.center
        self.get_logger().info('Received AprilTag detection: x=%d, y=%d' % (x, y))
        self.apriltag_positions.append((x, y))
        self.update_map()

    def update_map(self):
        plt.clf()
        ax = plt.gca()
        if self.yolo_positions:
            for (x, y, w, h) in self.yolo_boxes:
                rect = patches.Rectangle((x-w/2, y-h/2), w, h, linewidth=1, edgecolor='blue', facecolor='none')
                ax.add_patch(rect)
            x_coords, y_coords = zip(*self.yolo_positions)
            plt.scatter(x_coords, y_coords, color='blue', label='Block')
        if self.apriltag_positions:
            x_coords, y_coords = zip(*self.apriltag_positions)
            plt.scatter(x_coords, y_coords, color='red', label='Bittle')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xlim(0, 1280)
        plt.ylim(0, 960)
        plt.gca().invert_yaxis()
        plt.title('2D Map of Object Positions')
        plt.legend()

        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)

        image = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)

        map_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        self.map_publisher.publish(map_msg)

        

def main(args=None):
    rclpy.init(args=args)
    mapping_node = MappingNode()
    plt.ion()
    rclpy.spin(mapping_node)
    mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()