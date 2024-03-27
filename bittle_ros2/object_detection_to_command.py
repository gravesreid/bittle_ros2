import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection
from bittle_msgs.msg import Command 
from bittle_msgs.msg import State

cmd_dict = {1: 'kcrF', -1: 'kbk', 2: 'kcrL', 3: 'kcrR', 0: 'krest', 4: 'kpone',
             5: 'kptwo', 6: 'kpthree', 7: 'kpfour', 8: 'kcollectF', 9: 'kturn'}

class Driver(Node):
    def __init__(self):
        super().__init__('command_generation_node')
        self.subscription = self.create_subscription(
            Detection,
            '/detection_topic',
            self.detection_callback,
            1)
        # Create a publisher for the Command messages
        self.command_publisher = self.create_publisher(Command, 'serial_command_topic', 10)
        self.state_publisher = self.create_publisher(State, 'state_topic', 10)

        self.current_state = {
            'found_acorn': False,
            'found_black_pheromone': False,
            'found_white_pheromone': False,
            'searching': True,
            'collecting': False,
            'collected': False,
            'black_pheromones_dropped': 0,
            'white_pheromones_dropped': 0,
            'returning': False,
            'mission_complete': False,
            'last_command_sent': None
        }

    def detection_callback(self, msg):
        # Update state based on detection and publish command based on updated state
        self.current_state = self.analyze_detections(msg)
        command = self.decide_command(self.current_state)
        self.publish_command(command)
        self.publish_state()

    def publish_command(self, command):
        msg = Command()
        msg.cmd = command
        self.command_publisher.publish(msg)

def publish_state(self):
    msg = State()
    msg.found_acorn = self.current_state['found_acorn']
    msg.found_black_pheromone = self.current_state['found_black_pheromone']
    msg.found_white_pheromone = self.current_state['found_white_pheromone']
    msg.searching = self.current_state['searching']
    msg.collecting = self.current_state['collecting']
    msg.collected = self.current_state['collected']
    msg.black_pheromones_dropped = self.current_state['black_pheromones_dropped']
    msg.white_pheromones_dropped = self.current_state['white_pheromones_dropped']
    msg.returning = self.current_state['returning']
    msg.mission_complete = self.current_state['mission_complete']
    if self.current_state['last_command_sent'] is not None:
        msg.last_command_sent = self.current_state['last_command_sent']
    else:
        msg.last_command_sent = 'None'
    self.state_publisher.publish(msg)    

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
        elif self.current_state['searching'] and len(self.black_pheromone_list) > 0 and len(self.white_pheromone_list) ==0:
            self.current_state['found_black_pheromone'] = True

        

        return self.current_state

    def decide_command(self, state):
        # Decide the next command based on the state
        # Example simplified logic
        if state['found_acorn']:
            return self.follow_object(self.acorn_list)
        elif not state['found_acorn'] and state['found_black_pheromone']:
            return self.follow_object(self.black_pheromone_list)
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

