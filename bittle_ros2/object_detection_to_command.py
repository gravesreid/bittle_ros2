import rclpy
from rclpy.node import Node
from bittle_msgs.srv import SerialCommand
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection

cmd_dict = {1: 'kcrF', -1: 'kbk', 2: 'kcrL', 3: 'kcrR', 0: 'kbalance', 4: 'kpone',
             5: 'kptwo', 6: 'kpthree', 7: 'kpfour', 8: 'kcollectF', 9: 'kturn'}

class Driver(Node):
    def __init__(self):
        super().__init__('command_generation_node')
        self.subscription = self.create_subscription(
            Detection,
            '/detection_topic',
            self.detection_callback,
            1)
        self.service = self.create_service(
            SerialCommand,
            'get_next_command',
            self.get_next_command_callback
        )

        self.current_state = {
            'found_acorn': False,
            'searching': True,
            'collecting': False,
            'collected': False,
            'black_pheromones_dropped': 0,
            'white_pheromones_dropped': 0,
            'returning': False,
            'mission_complete': False,
            'last_command_sent': None
        }  # Simplified state representation

    def detection_callback(self, msg):
        # Update state based on detection
        # This should be where you analyze detections and update your state
        self.current_state = self.analyze_detections(msg)

    def get_next_command_callback(self, request, response):
        # Decide the next command based on current state
        if self.current_state is not None:
            command = self.decide_command(self.current_state)
            response.command = command
        else:
            response.command = 'No command'  # Default or idle command
        return response

    def analyze_detections(self, detection_msg):
        # Analyze detection messages and update state
        # Return a simplified state representation based on the latest detections
        self.get_logger().info("Received a /detection_topic message!")

        results = list(detection_msg.results) # returns a list with numeric labels for the objects 0: acorn 1: black pheromone 2: white pheromone
        xywhn_list = list(detection_msg.xywhn_list) #returns a list with the x, y, width, height, of each detection, ordered by the from the top of the frame to the bottom
        # make lists to store detection info
        self.acorn_list = []
        self.black_pheromone_list = []
        self.white_pheromone_list = []
        for i in range(len(results)): # for each detection, append the xywhn_list to the appropriate list
            if results[i] == 0:
                self.acorn_list.append(xywhn_list[(i*4):(4*(i+1))])
            elif results[i] == 1:
                self.black_pheromone_list.append(xywhn_list[(i*4):(4*(i+1))])
            elif results[i] == 2:
                self.white_pheromone_list.append(xywhn_list[(i*4):(4*(i+1))])       

        # based on the detection results, update the state
        if self.current_state['searching'] == True and len(self.acorn_list) > 0:
            self.current_state['searching'] = False
            self.current_state['found_acorn'] = True
        

        return self.current_state

    def decide_command(self, state):
        # Decide the next command based on the state
        # Example simplified logic
        if state['found_acorn']:
            return self.follow_object(self.acorn_list)
        else:
            return cmd_dict[0]
        
    def follow_object(self, detection_list):
        if len(detection_list) > 0:
            if detection_list[-1][0] > 0.6:
                cmd = cmd_dict[3]
            elif detection_list[-1][0] < 0.4:
                cmd = cmd_dict[2]
            else:
                cmd = cmd_dict[1]
        else:
            cmd = cmd_dict[0]
        return cmd

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

