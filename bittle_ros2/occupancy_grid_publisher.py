#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        qos_profile = QoSProfile(depth=10)

        # Subscribe to the camera feed
        self.sub_camera = self.create_subscription(
            Image,
            '/image_raw/compressed',
            self.camera_callback,
            qos_profile
        )
   

        # Publisher for the occupancy grid
        self.pub_grid = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        self.bridge = CvBridge()

        # Define some parameters for the map
        self.map_resolution = 0.01  # 1 cm/pixel in this naive example
        self.map_width = 320       # same width as your camera feed
        self.map_height = 240      # same height as your camera feed

        self.get_logger().info("OccupancyGridPublisher node has been started.")

    def camera_callback(self, msg: Image):
        # 1. Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Process the image to create a binary occupancy representation
        #    For example: grayscale -> threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # 3. Convert the binary image into occupancy data
        #    - Occupied (black) = 100
        #    - Free (white) = 0
        # Note: The threshold is reversed for demonstration (white => free, black => occupied)
        occupancy_data = []
        for row in thresh:
            for pixel in row:
                if pixel == 255:
                    occupancy_data.append(0)   # 0 = free
                else:
                    occupancy_data.append(100) # 100 = occupied

        # 4. Create the OccupancyGrid message
        grid_msg = OccupancyGrid()

        # Header
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'  # Or whatever frame you want

        # Map MetaData
        grid_msg.info = MapMetaData()
        grid_msg.info.resolution = self.map_resolution       # meters/pixel
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0  # no rotation

        # Occupancy data
        grid_msg.data = occupancy_data

        # 5. Publish the occupancy grid
        self.pub_grid.publish(grid_msg)
        self.get_logger().info("Published occupancy grid.")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()