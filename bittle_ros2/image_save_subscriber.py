import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Joy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class Save_Image(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.base_directory_p = r'/home/reid/data_collection/acorn'
        self.base_directory_n = r'/home/reid/data_collection/white_pheromone'
        os.makedirs(self.base_directory_p, exist_ok=True)  # Ensure the directories exist
        os.makedirs(self.base_directory_n, exist_ok=True)
        self.frame_counter_p = 0
        self.frame_counter_n = 0
        # self.prev_button_state = [0, 0]  # Initialize previous button state
        self.flag_8 = 0
        self.flag_9 = 0

        self.display_window_name = "Camera Image"
        cv2.namedWindow(self.display_window_name, cv2.WINDOW_NORMAL)

        self.subscription_1 = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            1)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Joy,  # Change message type to Joy
            '/joy',  # Change to the joystick topic
            self.capture_callback,
            10)
        self.subscription_2  # prevent unused variable warning

    def image_callback(self, data):
        try:
            self.get_logger().info('Receiving video frame')
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv2.imshow("camera", self.cv2_img)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def capture_callback(self, msg):
        try:
            buttons = msg.buttons
            if buttons[8] == 1 and self.flag_8 == 0:
                base_directory = self.base_directory_p
                frame_name = self.get_next_filename(base_directory, 'frame_', 'jpeg')
                image_path = os.path.join(base_directory, frame_name)
                cv2.imwrite(image_path, self.cv2_img)
                self.flag_8 = 1
            elif buttons[9] == 1 and self.flag_9 == 0:
                base_directory = self.base_directory_n
                frame_name = self.get_next_filename(base_directory, 'frame_', 'jpeg')
                image_path = os.path.join(base_directory, frame_name)
                cv2.imwrite(image_path, self.cv2_img)
                self.flag_9 = 1
            
            if buttons[8] == 0:
                self.flag_8 = 0

            if buttons[9] == 0:
                self.flag_9 = 0
            
        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def get_next_filename(self, directory, prefix, extension):
        files = os.listdir(directory)
        counter = 1
        while True:
            filename = f'{prefix}{counter}.{extension}'
            if filename not in files:
                return filename
            counter += 1


def main(args=None):
    rclpy.init(args=args)
    save_image_node = Save_Image()
    rclpy.spin(save_image_node)
    save_image_node.destroy_node()
    cv2.destroyAllWindows()  # Make sure to close OpenCV windows
    rclpy.shutdown()

if __name__ == '__main__':
    main()


