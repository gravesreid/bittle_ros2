import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Correct message type for compressed images
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import queue
import threading
import time
import cv2  # OpenCV library
from rclpy.executors import MultiThreadedExecutor

# define a class for displaying the video feed

class ImageDisplay:
    def __init__(self, max_queue_size=2):
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.display_thread = threading.Thread(target=self.run_display)
        self.display_thread.daemon = True
        self.display_thread.start()

    def run_display(self):
        while True:
            try:
                # Wait for the next frame with a timeout to allow checking for exit conditions
                frame = self.frame_queue.get(timeout=0.01)
                cv2.imshow("camera", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
            except queue.Empty:
                # The timeout allows the loop to check for exit conditions regularly
                continue

    def update_frame(self, frame):
        if not self.frame_queue.full():
            # If the queue is not full, add the new frame
            try:
                self.frame_queue.put(frame, block=False)
            except queue.Full:
                pass  # If the queue becomes full between the check and the put, ignore this frame
        else:
            # If the queue is full, discard the oldest frame and add the new one
            try:
                self.frame_queue.get(block=False)  # Discard the oldest frame
                self.frame_queue.put(frame, block=False)  # Add the new frame
            except queue.Empty:
                # If the queue emptied since checking, this exception will catch it
                pass

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.listener_callback, 1)
        self.image_display = ImageDisplay()  # Create an instance of the ImageDisplay class

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.image_display.update_frame(current_frame)  # Update the frame to be displayed

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    rclpy.spin(image_subscriber, executor=executor)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()