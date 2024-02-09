#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # Ensure you have this package installed

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/reid/autonomous-bittle/vision-based-command/acorns_and_decahedrons_v1/best.pt')  # Update the path to your model
        self.model.conf = 0.5  # Set confidence threshold
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Perform object detection
        results = self.model([current_frame])  # Ensure this matches how your model expects input
        print(len(results))
                        # handling detections
        if len(results) > 0:
            result_list = (results[0].boxes.cls).cpu().tolist()
            xywhn_list =  (results[0].boxes.xyxyn).cpu().tolist()
            # make list for acorn detection locations
            acorn_locations = []
            # make list for pheromone detection locations
            pheromone_locations = []
            for i in range(len(result_list)):
                if int(result_list[i]) == 0:
                    acorn_locations.append(xywhn_list[i])
                elif int(result_list[i]) == 1:
                    pheromone_locations.append(xywhn_list[i])
            # sort acorn locations by y value
            acorn_locations.sort(key=lambda x: x[1], reverse=True)
            # sort pheromone locations by y value
            pheromone_locations.sort(key=lambda x: x[1], reverse=True)
            
            # make list of center points of acorn locations
            acorn_centers = []
            for i in range(len(acorn_locations)):
                acorn_centers.append(((acorn_locations[i][0] + acorn_locations[i][2])/2, (acorn_locations[i][1] + acorn_locations[i][3])/2))
            print("acorn centers: ", acorn_centers, "size: ", len(acorn_centers))
            # make list of center points of pheromone locations
            pheromone_centers = []
            for i in range(len(pheromone_locations)):
                pheromone_centers.append(((pheromone_locations[i][0] + pheromone_locations[i][2])/2, (pheromone_locations[i][1] + pheromone_locations[i][3])/2))
            print("pheromone centers: ", pheromone_centers, "size: ", len(pheromone_centers))
        #end handling detections
            # send commands based on detections
            # if there are acorns detected
           # if len(acorn_centers) > 0:
           #     twist_msg = Twist()
           #     twist_msg.linear.x = .5
           #     twist_msg.angular.z = 0.0
           #     self.cmd_vel_publisher.publish(twist_msg)
           #     self._logger().info("Moving forward")
            
        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv5 Detection", annotated_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
