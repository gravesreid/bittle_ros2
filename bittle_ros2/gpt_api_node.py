import rclpy
from rclpy.node import Node
from bittle_msgs.srv import CapturePhoto, ExecuteCommand
from cv_bridge import CvBridge
import cv2
import os
from openai import OpenAI
import base64
import threading
import time
import json
from bittle_msgs.msg import Detection
from bittle_msgs.action import MoveToGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
import re 

api_loc = "/home/reid/ros2_ws/gpt_key.txt"

if not os.path.isfile(api_loc):
    raise FileNotFoundError(f"API key file not found: {api_loc}")

with open(api_loc, 'r') as f:
    api_key = f.readline().strip()

class GPT_Client(Node):

    def __init__(self):
        super().__init__('gpt_client')
        self._action_client = ActionClient(
            self,
            MoveToGrid,
            'move_to_grid'
        )
        self.detection_subscription = self.create_subscription(
            Detection,
            'detection',
            self.detection_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        self.image_subscription = self.create_subscription(
            Image,
            'detection_image',
            self.image_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        with open("action_prompt_0.txt", "r") as f:
            self.action_prompt = f.read()
        self.current_heading = None
        self.current_position = None
        self.detections = []
        self.image_path = 'input_image.jpg'
        self.current_image = None

    def image_callback(self, msg):
        bridge = CvBridge()
        self.current_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite("input_image.jpg", self.current_image)
        self.send_goal_grid()

    def get_caption(self, action_prompt):
        client = OpenAI(api_key=api_key)
        with open(self.image_path, "rb") as f:
            encoded_image = base64.b64encode(f.read()).decode('utf-8')

        messages = [
            {
                "role": "user",
                "content": action_prompt,
            },
            {
                "role": "function",
                "name": "image_input",
                "content": f"data:image/jpeg;base64,{encoded_image}"
            }
        ]

        completion = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            max_tokens=300
        )

        # Extract command from tool call response


        message = completion.choices[0].message.content
        # Find the first occurrence of the pattern [RC]
        self.get_logger().info(f"Message: {message}")
        start_index = message.find("[")
        end_index = message.find("]", start_index)

        goal_cell = None

        if start_index != -1 and end_index != -1:
            # Extract the content between the brackets
            goal_cell = message[start_index + 1:end_index]
            
            # Validate the content
            if len(goal_cell) == 2 and goal_cell[0] in "ABCD" and goal_cell[1] in "1234":
                print(f"Extracted goal cell type: {goal_cell.__class__}")
                self.get_logger().info(f"Extracted goal cell: {goal_cell}")
            else:
                self.get_logger().info("Pattern found but does not match expected format [RC].")

        if goal_cell:
                self.get_logger().info(f"Goal cell: {goal_cell}")
                return f"{goal_cell}"
        else:
            self.get_logger().error("No command received from model")
            return ""

def main(args=None):
    rclpy.init(args=args)
    gpt_client = GPT_Client()
    rclpy.spin(gpt_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()