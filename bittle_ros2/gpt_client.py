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
            'move_to_grid',
            self.send_goal_grid,
            callback_group=ReentrantCallbackGroup()
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
        
        with open("action_prompt.txt", "r") as f:
            self.action_prompt = f.read()
        self.current_heading = None
        self.current_position = None
        self.detections = []
        self.image_path = 'input_image.jpg'

    def send_goal_grid(self):
        current_image = self.image_callback()
        cv2.imwrite(self.image_path, current_image)
        action_prompt = self.action_prompt
        if self.detections:
            # add current detections to action prompt
            for detection in self.detections:
                class_name, grid_square, center = detection
                action_prompt += f" {class_name} at {grid_square} with center {center}"
        goal_msg = MoveToGrid.Goal()
        goal_msg.target_square = self.get_caption(action_prompt)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._action_client.send_goal_async(goal_msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def detection_callback(self, msg):
        self.current_heading = msg.april_tag_orientation
        self.current_position = msg.april_tag_location
        centers = list(zip(*(iter(msg.center),) * 2))

        for class_name, grid_square, center in zip(msg.class_names, msg.grid_squares, centers):
            center_x, center_y = center[0], center[1]
            self.detections.append((class_name, grid_square, (center_x, center_y)))


        self.get_logger().info(f'class names: {msg.class_names}, grid squares: {msg.grid_squares}, Centers: {centers}')
        self.get_logger().info(f'Received detections: {self.detections}')

    def image_callback(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite("input_image.jpg", image)
        return image

    def get_caption(self, action_prompt):
        client = OpenAI(api_key=api_key)
        with open(self.image_path, "rb") as f:
            encoded_image = base64.b64encode(f.read()).decode('utf-8')

        tools = [
            {
                "type": "function",
                "function": {
                    "name": "move_to_grid_square",
                    "description": "Move the robot to the specified grid square",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "cell": {
                                "type": "string",
                                "description": "The cell to move to, e.g. 'A1'",
                            },
                        },
                        "required": ["cell"],
                    },
                }
            }
        ]

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
            tools=tools,
            tool_choice="auto",
            max_tokens=300
        )

        # Extract command from tool call response
        tool_calls = completion.choices[0].message.tool_calls
        self.get_logger().info(f"Message: {completion.choices[0].message.content}")
        if tool_calls:
                for tool_call in tool_calls:
                    function_args = json.loads(tool_call.function.arguments)
                    self.get_logger().info(f"Function args: {function_args}")
                    cell = function_args["cell"]
                    self.get_logger().info(f"Command received: {cell}")
                    self.send_goal_grid(cell)
        else:
            self.get_logger().error("No command received from model")

def main(args=None):
    rclpy.init(args=args)
    gpt_client = GPT_Client()
    rclpy.spin(gpt_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()