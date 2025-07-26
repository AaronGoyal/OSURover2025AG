import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
from time import time, sleep
import math
import can
import struct

MAX_ACCEL = 0.05

GEAR_RATIO = 50
RPS_FACTOR = 2
#THIS SHOULD BE 2, 4.4 IS FOR THE SPEED LOL
#RPS_FACTOR = 4.4
BUS = can.interface.Bus("can0", interface="socketcan")

VEL_RAMP = 4*GEAR_RATIO*RPS_FACTOR
LEFT_NODES = [0,2,4]
RIGHT_NODES = [1,3,5]
NODES = [0,1,2,3,4,5]

WATCHDOG_TIMEOUT = 3.0

class DriveCanControlNode(Node):
    def __init__(self):
        super().__init__('drive_can_control_node')

        #Initialize class variables for desired velocities
        self.linear_velocity = 0
        self.angular_velocity = 0

        while not (BUS.recv(timeout=0) is None): pass
 
        self.setup_controller()

        self.groundstation_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/ground_station_drive',  # Topic where joy messages are published
            self.groundstation_drive_command_callback,
            1
        )
        self.iris_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/iris_drive',  # Topic where joy messages are published
            self.iris_drive_command_callback,
            1
        )

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.last_message_time = time()

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x)) - 0.5
        
    def setup_controller(self):
        for node_id in NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
            ))
          
            #Set velocity ramp control mode
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0b), 
            data=struct.pack('<II', 2,2),
            is_extended_id=False
            ))

            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04), 
            data=struct.pack('<BHBf', 1,403,0,VEL_RAMP),
            is_extended_id=False
            ))

    def timer_callback(self):
        #Watchdog to make sure the drive doesn't spin out of control if connection is lost/messages stop being recieved
        self.get_logger().info(f"Time: {time()}, Last Message Time: {self.last_message_time}")
        if time() >= self.last_message_time+2:    
            self.linear_velocity = 0.0  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = 0.0  # Right joystick horizontal axis (turning)
        
        #This handles drivetrain saturation (sigmoid function), and control mixing
        self.normalize_drive_commands()

    def normalize_drive_commands(self):
        
        #Umm actually the sigmoid does something here now henry :)
        left_velocity = -1 * self.sigmoid(self.linear_velocity - self.angular_velocity) * GEAR_RATIO * RPS_FACTOR 
        right_velocity = self.sigmoid(self.linear_velocity + self.angular_velocity) * GEAR_RATIO * RPS_FACTOR
        
        #Logic for sending velocity through can HERE
        for node_id in LEFT_NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', left_velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
            ))

        for node_id in RIGHT_NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', right_velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
            ))

    def groundstation_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        if msg.controller_present:
            #Update the desired velocities
            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)         self.normalize_drive_commands() 
            
            self.last_message_time = time() #Only update the watchdog timer if we recieve a message and a controller is present
        
    def iris_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        if msg.controller_present:

            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
            self.normalize_drive_commands()
            # You may want to scale these velocities to suit your needs (e.g., make them faster/slower)
            self.last_message_time = time()


def main(args=None):
    rclpy.init(args=args)
    node = DriveCanControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
