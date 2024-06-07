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

api_loc = "/home/reid/ros2_ws/gpt_key.txt"

if not os.path.isfile(api_loc):
    raise FileNotFoundError(f"API key file not found: {api_loc}")

with open(api_loc, 'r') as f:
    api_key = f.readline().strip()

class PhotoClient(Node):

    def __init__(self):
        super().__init__('photo_client_node')
        self.client = self.create_client(CapturePhoto, 'capture_photo')
        self.command_client = self.create_client(ExecuteCommand, 'execute_command')
        time.sleep(10)  # Add a delay to ensure the services have enough time to start
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('capture_photo service not available, waiting...')
        self.get_logger().info('Services available, continuing...')
        self.request = CapturePhoto.Request()
        self.bridge = CvBridge()
        self.send_request()

    def send_request(self):
        self.get_logger().info('Sending request to capture photo')
        future = self.client.call_async(self.request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received image')
            image = self.bridge.imgmsg_to_cv2(response.image, "bgr8")
            cv2.imwrite("input_image.jpg", image)
            self.get_caption(image)
            threading.Thread(target=self.display_image, args=(image,)).start()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def display_image(self, image):
        cv2.imshow("Captured Image", image)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

    def get_caption(self, image):
        client = OpenAI(api_key=api_key)
        with open("input_image.jpg", "rb") as f:
            encoded_image = base64.b64encode(f.read()).decode('utf-8')

        tools = [
            {
                "type": "function",
                "function": {
                    "name": "send_command",
                    "description": "Send a command to the robot",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "command": {
                                "type": "string",
                                "description": "The command to execute, e.g., rest, fwd, back, left, right",
                            },
                        },
                        "required": ["command"],
                    },
                }
            }
        ]

        messages = [
            {
                "role": "user",
                "content": "You are a robot and need to explore the room until you find an exit after you exit the room, rest. Choose a command to execute from rest, forward, back, spin in place left or right, and turn left or right, eg: rest, fwd, spinleft, spinright, left, right, or back",
            },
            {
                "role": "function",
                "name": "image_input",
                "content": f"data:image/jpeg;base64,{encoded_image}"
            }
        ]

        completion = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            tools=tools,
            tool_choice="auto"
        )

        # Extract command from tool call response
        tool_calls = completion.choices[0].message.tool_calls
        self.get_logger().info(f"{completion.choices[0].message.content}")
        if tool_calls:
                for tool_call in tool_calls:
                    function_args = json.loads(tool_call.function.arguments)
                    self.get_logger().info(f"Function args: {function_args}")
                    command = function_args["command"]
                    self.get_logger().info(f"Command received: {command}")
                    self.send_command(command)
        else:
            self.get_logger().error("No command received from model")

    def send_command(self, command):
        self.get_logger().info(f"Sending command: {command}")
        request = ExecuteCommand.Request()
        request.command = command
        future = self.command_client.call_async(request)
        future.add_done_callback(self.command_response_callback)

    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Command executed successfully')
            else:
                self.get_logger().error('Command execution failed')
            self.send_request()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    photo_client = PhotoClient()
    rclpy.spin(photo_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




