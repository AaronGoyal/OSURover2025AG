import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import can 
import struct
from time import time, sleep

from std_msgs.msg import String


class GripperCanControl(Node):

    def __init__(self):
        super().__init__('gripper_can_control')

        self.last_message_time = time()
        #odrive params
        self.node_id = 6
        self.axis = 0

        self.vel = 0.0
        self.vel_setpoint = 10.0
        self.torq_setpoint = 0.03124
        self.pos_setpoint = 0.0

        self.vel_limit = 24.0
        self.vel_ramp_rate = 10.0
        self.current_threshold = 4.0
        self.accel_limit = 5.0
        self.deccel_limit = 5.0

        self.current = 0.0
        self.current_pos = 0.0
        self.current_vel = 0.0

        self.mode = 0

        #Homing Params
        self.is_homed = False
        self.home_current_threshold = 4.0
        self.home_vel = 2.0
        self.home_pos = 0.0
        self.home_offset = -1.0
    

        #joy Mappings
        self.open_button = 1
        self.close_button = 2

        #setup can
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.can_timer = self.create_timer(0.005, self.read_can)
        while not (self.bus.recv(timeout=0) is None): pass
        #set up joy
        self.create_subscription(Joy, '/joy', self.joy_callback, 1)

        #create time out timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        #Initialize Odrive and Gripper position 
        self.get_logger().info("Starting Setup of Odrive")
        self.setup_controller()

    def timer_callback(self):
        #complete Homing
        if not self.is_homed:
            self.get_logger().info("homing")
            self.set_mode(2)
            self.send_velocity(self.home_vel)
        #timeout period of half sec
        elif abs(self.last_message_time - time()) > 0.5 and self.mode != 1:
            #self.get_logger().info("No inputs")
            if self.mode != 3:
                self.get_logger().info("here")
                self.set_mode(3)
                self.send_position(pos=self.current_pos)
            elif abs(self.current_pos - self.pos_setpoint) > 0.02 and self.current_vel < 0.01:
                self.get_logger().info("out of position")
                self.send_position(self.pos_setpoint)
        #handle going to torque mode
        # **TOO DO** Handle object Slipping
        elif self.mode == 1:
            if self.current < self.current_threshold:
                self.send_torque(self.torq_setpoint)
        #Handle Velocity Mode
        elif self.mode == 2:
            self.send_velocity(self.vel)
        #Handle Position mode:
        elif self.mode == 3:
            if abs(self.current_pos - self.pos_setpoint) > 0.02 and self.current_vel < 0.01:
                self.send_position(self.pos_setpoint)

    def joy_callback(self, msg):

        self.last_message_time = time()
        buttons = msg.buttons
        if self.is_homed:
            if buttons[self.open_button]: 
                if self.current_pos >= self.home_pos:
                    self.set_mode(3)
                    self.pos_setpoint = self.home_pos
                else:
                    self.set_mode(2)
                    self.vel = self.vel_setpoint

            elif buttons[self.close_button]: 
                if self.current > self.current_threshold:
                    self.set_mode(1)
                else: 
                    self.set_mode(2)
                    self.vel = -self.vel_setpoint
            else:
                self.set_mode(3)
                self.pos_setpoint = self.current_pos

    def setup_controller(self):
        self.set_mode(self.mode)
        self.home()
        self.get_logger().info("Finished Setup")

    def home(self):
        self.get_logger().info("Start Homing Sequence")
        self.set_mode(2) #enable closed loop ramped velocity mode
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            #self.get_logger().info("In while loop")
            if self.current > self.home_current_threshold:
                self.home_pos = self.current_pos + self.home_offset
                self.is_homed = True
                self.set_mode(3)
                self.send_position(self.home_pos)
                break
            sleep(0.01)
        self.get_logger().info(f"Homed, Position: {self.home_pos}")
        self.last_message_time = time()

    def send_torque(self, torq):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0e),
            data=struct.pack('<f', torq)
        ))

    def send_velocity(self, vel):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', vel, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
        ))
        #self.get_logger().info(f"Sending Velocity: {vel}")  
        
    def send_position(self, pos):
        self.bus.send(can.Message(
            arbitration_id = (self.node_id << 5 | 0x0c),
            data = struct.pack('<fHH', pos, 0, 0),
            is_extended_id = False
        ))
        self.pos_setpoint = pos

    def set_mode(self, mode):
        if self.mode != mode:
            self.get_logger().info(f"mode: {mode}")
            match mode:
                case 0:
                    #Set Cloosed Loop control
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                        data=struct.pack('<I', 1), # 8: AxisState.IDLE
                        is_extended_id=False
                    ))
                    self.mode = 0

                case 1:
                    #Set Cloosed Loop control
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                        is_extended_id=False
                    ))
                    ##set Torq control and passthrough
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x0b), 
                        data=struct.pack('<II', 1,1),
                        is_extended_id=False
                    ))
                    self.mode = 1
                case 2:
                    #Set Cloosed Loop control
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                        is_extended_id=False
                    ))
                    #Set velocity ramp control mode
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x0b), 
                        data=struct.pack('<II', 2,2),
                        is_extended_id=False
                    ))

                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x04), 
                        data=struct.pack('<BHBf', 1, 396,0, self.vel_ramp_rate), #403 - 0.6.10, 396 - 0.6.9-1
                        is_extended_id=False
                    ))
                    self.mode = 2
                case 3:
                    #Set Cloosed Loop control
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                        is_extended_id=False
                    ))
                    #set command mode position, input mode trap_traj
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x0b),
                        data=struct.pack('<II', 3, 5),
                        is_extended_id= False
                    ))

                    #Traj Velo Limit
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x11),
                        data=struct.pack('<f', self.vel_setpoint),
                        is_extended_id= False
                    ))

                    #trap accel limit
                    self.bus.send(can.Message(
                        arbitration_id=(self.node_id << 5 | 0x12),
                        data=struct.pack('<ff', self.accel_limit, self.deccel_limit),
                        is_extended_id= False
                    ))
                    self.get_logger().info("set trap_traj mode")
                    self.mode = 3

    #Define a callback for watching can messages:
    def read_can(self):
        #Get a buffer of stored messages and iterate over it
        can_msgs = self.get_can_buffer()

        for can_msg in can_msgs:

            #Don't do anything if a None message gets through
            if can_msg == None:
                continue

            #Masks for getting the command and node ids
            node_mask = (1 << 6) - 1 
            cmd_mask = (1 << 5) - 1

            #First pull out and save the node ID and command ID:
            node_id = (can_msg.arbitration_id >> 5) & node_mask
            cmd_id = can_msg.arbitration_id & cmd_mask
                
            #Probably should check for the RTR bit in case someone specifically requests data while this is running
            #if can_msg.rtr: #This means the message is a request -> no data
            #	continue
            if node_id == self.node_id:
            #Use match with the command id to unpack the message correctly:
                result = {} #Assign this an empty dict in case the match falls thru
                match cmd_id:
                    case 0x01: #Heartbeat
                        pass	
                    case 0x09: #Encoder Estimate of Position/Velocity
                        pos_estimate, vel_estimate = struct.unpack('<ff', bytes(can_msg.data))
                        self.get_logger().info(f'position: {pos_estimate}')
                        self.current_pos = pos_estimate
                        self.current_vel = vel_estimate
                    
                    case 0x14: #Q Axis motor current set/measured
                        iq_set, iq_measured = struct.unpack('<ff', bytes(can_msg.data))
                        self.current = iq_measured

                        self.get_logger().info(f"Current: {iq_measured}, Set: {iq_set}")

                    #case 0x1c: #Torque Target/Estimate

    #Abstraction for getting all can messages currently in the buffer:
    def get_can_buffer(self):

        #Return a max of 1000 msgs, that way we don't miss publishing
        max_return_msgs = 1000

        can_msgs = []
        
        msg_count = 0

        while True:
            #Check max msgs first (that way we don't lose a msg)
            if msg_count == max_return_msgs-1:
                break
        
            #Read the msg
            can_msg = self.bus.recv(timeout=0)
        
            #We get to the last msg if reading it returns None:
            if can_msg == None:
                break

            #Add them to a list and count the number:
            #can_msgs[msg_count] = can_msg
            can_msgs.append(can_msg)
            msg_count += 1

        #Return list of msgs
        return can_msgs

    def destroy_node(self):
        # Set axis to IDLE before shutdown
        self.get_logger().info("Shutting down... setting ODrive to IDLE")

        try:
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x07),  # Set_Axis_State
                data=struct.pack('<I', 1),  # 1 = AXIS_STATE_IDLE
                is_extended_id=False
            ))
        except Exception as e:
            self.get_logger().error(f"Failed to send IDLE to ODrive: {e}")

        # Call the parent class's destroy_node to clean up timers/subs
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperCanControl()

    rclpy.spin(gripper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()