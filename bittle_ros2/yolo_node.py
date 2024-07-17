import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bittle_msgs.msg import Detection
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading
import apriltag

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
        self.model = YOLO('/home/reid/Projects/Bittle/Bittle_LLM/yolo/runs/detect/train3/weights/best.pt')  # Replace with your custom model path if needed

        self.colors = {
            0: (255, 0, 0),   # Color for class 0 (e.g., red)
            1: (0, 0, 255),   # Color for class 1 (e.g., blue)
        }

        # Define grid parameters
        self.pxstep = 160
        self.pystep = 120

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            return
        
        # april tag stuff
        self.detector = apriltag.Detector()
        self.tag_size = 0.0275  # 27.5 mm

        self.camera_matrix = np.array([[2.51424135e+03, 0.00000000e+00, 3.04989934e+02],
            [0.00000000e+00, 2.51320448e+03, 2.35529666e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist_coeff = np.array([
            [ 3.39495079e+00, -1.47533299e+02, -4.24259153e-03, -1.07502528e-02,
            5.27406271e+02]])
        

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

            # april tag detection
            cX, cY, yaw = None, None, None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tag_results = self.detector.detect(gray)
            for tag in tag_results:
                (ptA, ptB, ptC, ptD) = tag.corners
                ptA = (int(ptA[0]), int(ptA[1]))
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
                cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
                cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
                cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
                (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                tagFamily = tag.tag_family.decode("utf-8")
                tagID = tag.tag_id
                object_points = np.array([
                    [-self.tag_size / 2, -self.tag_size / 2, 0],
                    [self.tag_size / 2, -self.tag_size / 2, 0],
                    [self.tag_size / 2, self.tag_size / 2, 0],
                    [-self.tag_size / 2, self.tag_size / 2, 0]
                ])
                image_points = np.array([ptA, ptB, ptC, ptD], dtype="double")
                _, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeff)
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                position = tvec.flatten()
                orientation = rotation_matrix.flatten()
                yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])*180/np.pi
                yaw = -np.float64(yaw)

            # Draw grid on the frame
            draw_grid(frame, pxstep=self.pxstep, pystep=self.pystep)
            print(f"Frame shape: {frame.shape}")
            
            # Create a copy of the frame with grids for saving without bounding boxes
            clean_frame_with_grids = frame.copy()

            # Run YOLOv8 object detection
            results = self.model(frame)

            # List to store detection information
            detections = []

            msg = Detection()
            msg.results = []
            msg.center = []
            msg.grid_squares = []
            msg.class_names = []
            msg.april_tag_location = []
            if cX is not None and cY is not None:
                msg.april_tag_location.extend([cX, cY])
                msg.april_tag_orientation = yaw

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
                    msg.center.extend([center_x, center_y])
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
