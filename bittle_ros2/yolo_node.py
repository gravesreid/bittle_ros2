import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bittle_msgs.msg import Detection
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading

# Function to draw grid lines on the image
def draw_grid(img, line_color=(0, 255, 0), thickness=1, type_=cv2.LINE_AA, pxstep=180, pystep=145):
    x = pxstep
    y = pystep
    while x < img.shape[1]:
        cv2.line(img, (x, 0), (x, img.shape[0]), color=line_color, lineType=type_, thickness=thickness)
        x += pxstep

    while y < img.shape[0]:
        cv2.line(img, (0, y), (img.shape[1], y), color=line_color, lineType=type_, thickness=thickness)
        y += pystep

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.publisher_ = self.create_publisher(Detection, 'detection', 10)
        self.image_publisher_ = self.create_publisher(Image, 'detection_image', 10)
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/home/reid/Projects/Bittle/Bittle_LLM/yolo/runs/detect/train2/weights/best.pt')  # Replace with your custom model path if needed

        self.colors = {
            0: (255, 0, 0),   # Color for class 0 (e.g., red)
            1: (0, 0, 255),   # Color for class 1 (e.g., blue)
        }

        # Define grid parameters
        self.pxstep = 160
        self.pystep = 120

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            return

        # Start the detection thread
        self.detection_thread = threading.Thread(target=self.run)
        self.detection_thread.daemon = True
        self.detection_thread.start()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: Failed to capture image.")
                break

            # Draw grid on the frame
            draw_grid(frame, pxstep=self.pxstep, pystep=self.pystep)
            
            # Create a copy of the frame with grids for saving without bounding boxes
            clean_frame_with_grids = frame.copy()

            # Run YOLOv8 object detection
            results = self.model(frame)

            # List to store detection information
            detections = []

            msg = Detection()
            msg.results = []
            msg.xywhn_list = []
            msg.grid_squares = []
            msg.class_names = []

            # Process YOLOv8 results
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = box.conf[0]
                    class_id = int(box.cls[0])
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    grid_square = self.find_row_col(center_x, center_y)
                    
                    detection_info = {
                        "class_id": self.model.names[class_id],
                        "coordinates": (x1, y1, x2, y2),
                        "center": (center_x, center_y),
                        "grid_square": grid_square,
                        "confidence": confidence.item()  # Convert tensor to a regular float
                    }
                    detections.append(detection_info)
                    
                    # Populate the Detection message
                    msg.results.append(class_id)
                    msg.xywhn_list.extend([center_x, center_y, x2-x1, y2-y1])
                    msg.grid_squares.append(grid_square)
                    msg.class_names.append(self.model.names[class_id])

                    color = self.colors.get(class_id, (255, 255, 255))  # Default to white if class_id is not in colors
                    label = f"{self.model.names[class_id]}: {confidence:.2f} ({grid_square})"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Publish detection message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published detection: {detections}")

            # Publish the image with detections
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher_.publish(image_msg)

            # Display the resulting frame
            cv2.imshow('Webcam with Grid and YOLOv8', frame)

            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def find_row_col(self, x, y):
        def find_col(x):
            if x < self.pxstep:
                return "1"
            elif x < 2 * self.pxstep:
                return "2"
            elif x < 3 * self.pxstep:
                return "3"
            else:
                return "4"
        if y < self.pystep:
            return "A" + find_col(x)
        elif y < 2 * self.pystep:
            return "B" + find_col(x)
        elif y < 3 * self.pystep:
            return "C" + find_col(x)
        else:
            return "D" + find_col(x)

def main(args=None):
    rclpy.init(args=args)
    yolo_detection_node = YoloDetectionNode()
    rclpy.spin(yolo_detection_node)
    yolo_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
