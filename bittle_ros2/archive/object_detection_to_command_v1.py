import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection
from bittle_msgs.msg import Command 
from bittle_msgs.msg import State
import time

cmd_dict = {'forward': 'kcrF', 'back': 'kbk', 'left': 'kcrL', 'right': 'kcrR', 'rest': 'krest', 'black1': 'kpone',
             'black2': 'kptwo', 'white1': 'kpthree', 'white2': 'kpfour', 'collect': 'kcollectF', 'spinLeft': 'kvtL', 
             'spinRight': 'kvtR', 'buttUp': 'kbuttUp'}

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

    def process_detections(self):
        if not hasattr(self, 'latest_detection_msg'):
            self.get_logger().info("No detection message received yet.")
            return
        msg = self.latest_detection_msg
        

    def publish_command(self, command):
        if isinstance(command, list):
            for cmd, delay in command:
                print(f"Sending command: {cmd}")
                print(f"Delaying for {delay} seconds")
                msg = Command()
                msg.cmd = cmd
                self.command_publisher.publish(msg)
                time.sleep(delay)
        else:
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
            last_command = self.current_state['last_command_sent']
            msg.last_command_sent = str(last_command) 
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
        # if we are searching and find an acorn, we are no longer searching
        if self.current_state['searching'] == True and len(self.acorn_list) > 0:
            self.current_state['searching'] = False
            self.current_state['found_acorn'] = True
        elif not self.current_state['found_black_pheromone'] and len(self.black_pheromone_list) > 0:
            self.current_state['found_black_pheromone'] = True
        elif not self.current_state['found_white_pheromone'] and len(self.white_pheromone_list) > 0:
            self.current_state['found_white_pheromone'] = True
        elif len(self.acorn_list) > 0 and self.acorn_list[-1][1] > 0.9:
            print("acorn position: ", self.acorn_list[-1][1])
            self.current_state['collecting'] = True
        
        return self.current_state

    def decide_command(self, state):
        # if we found an acorn, that's all we need care about
        if state['found_acorn'] and not state['collecting']: 
            cmd = self.follow_object(self.acorn_list)
        # if we haven't found an acorn, but we have found black pheromones, follow them
        elif not state['found_acorn'] and state['found_black_pheromone']:
            cmd = self.follow_object(self.black_pheromone_list)
        # if we haven't found an acorn or black pheromones, but we have found white pheromones, follow them
        elif (not state['found_acorn'] and not state['found_black_pheromone'] and state['found_white_pheromone']):
            cmd = self.follow_object(self.white_pheromone_list)
        elif state['collecting']:
            cmd = self.collect_acorn(self.acorn_list)

        # if we haven't found anything, stop and wait for a detection
        else:
            cmd = cmd_dict['rest']
        self.current_state['last_command_sent'] = cmd
        return cmd
        
    def follow_object(self, detection_list):
        if len(detection_list) > 0:
            if detection_list[-1][0] > 0.7:
                cmd = cmd_dict['right']
            elif detection_list[-1][0] < 0.3:
                cmd = cmd_dict['left']
            else:
                cmd = cmd_dict['forward']
        else:
            cmd = cmd_dict['rest']
        return cmd
    
    def collect_acorn(self, detection_list):
       initial_detection_count = len(detection_list)
       commands = []
       while not self.current_state['collected']:
        #if not detection_list:
            #cmd = cmd_dict['rest']
            #print("No detection found. Exiting...")
            #break
        
        x_position, y_position = detection_list[-1][:2]
        
        # Adjust the robot's x position by spinning.
        if x_position > 0.7:
            print("acorn is to the right, spinning right")
            commands.append([cmd_dict['spinRight'], .25])
            commands.append([cmd_dict['buttUp'], 0])
        elif x_position < 0.3:
            print("acorn is to the left, spinning left")
            commands.append([cmd_dict['spinLeft'], .25])
            commands.append([cmd_dict['buttUp'], 0])
        
        # Adjust the robot's y position by moving backward or forward.
        if y_position < 0.5:
            print("acorn is below, moving forward")
            commands.append([cmd_dict['forward'], .5])
            commands.append([cmd_dict['buttUp'], 0])
        elif y_position > 0.8:
            print("acorn is above, moving backward")
            commands.append([cmd_dict['back'], .5])
            commands.append([cmd_dict['buttUp'], 0])
        
        # If the x and y positions are within acceptable ranges, attempt to collect the acorn.
        if 0.3 <= x_position <= 0.7 and 0.5 <= y_position <= 0.8:
            print("acorn is in the center, collecting")
            commands.append([cmd_dict['collect'], 1])
            commands.append([cmd_dict['buttUp'], 0])
            # Check if the acorn was successfully collected.
            if len(detection_list) < initial_detection_count:
                self.current_state['collected'] = True
                break
            else:
                # If the acorn was not collected, reassess the position and try again.
                commands.append([cmd_dict['buttUp'], 0])
                initial_detection_count = len(detection_list)

        return commands

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

