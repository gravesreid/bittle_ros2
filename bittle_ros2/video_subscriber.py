import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from multiprocessing import Process, Queue, Value
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor

def display_frames(frame_queue, stop_signal):
    while not stop_signal.value:
        if not frame_queue.empty():
            frame = frame_queue.get()
            if frame is None:
                break
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) == ord('q'):
                stop_signal.value = 1
                break

class ImageSubscriber(Node):
    def __init__(self, frame_queue, stop_signal):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.frame_queue = frame_queue
        self.stop_signal = stop_signal
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.listener_callback, 1)

    def listener_callback(self, data):
        try:
            current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
            if not self.frame_queue.full():
                self.frame_queue.put(current_frame)
            else:
                self.get_logger().warn("Frame queue is full. Dropping frame.")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    frame_queue = Queue(maxsize=10)
    stop_signal = Value('i', 0)
    display_process = Process(target=display_frames, args=(frame_queue, stop_signal))
    display_process.start()

    rclpy.init(args=args)
    image_subscriber = ImageSubscriber(frame_queue, stop_signal)
    executor = MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    try:
        executor.spin()
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        stop_signal.value = 1
        frame_queue.put(None)  # Signal the display process to exit
        display_process.join()

if __name__ == '__main__':
    main()











