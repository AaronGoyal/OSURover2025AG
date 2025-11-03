import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from geometry_msgs.msg import TwistStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState



#moveit stuff
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions

from sensor_msgs.msg import Joy

from controller_manager_msgs.srv import SwitchController


#python stuff
import time

class SquareMakingController(Node):

    def __init__(self):
        super().__init__('square_maker')
        self.get_clock().now()
        self.cb_group = MutuallyExclusiveCallbackGroup()


        #clients
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')

        self.controller_client = self.make_client(SwitchController, '/controller_manager/switch_controller')
        self.configure_servo_cli = self.make_client(SetParameters, '/servo_node/set_parameters')

        #Publishers
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.joy_pub_ = self.create_publisher(Joy, '/joy2', 1)

        #subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)

        #timers
        self.timer = self.create_timer(0.002, self.timer_callback)

        #state params
        self.state = "start"
        self.latest_joint_state = None
        self.move_success = False
        self.sent_goal = False
        self.servo = True

        #define scan params
        self.sequence = {"down":"right", "right":"up", "up":"left", "left":"down"}
        self.dir_def = {"down":[0.0,-0.05], "right":[-0.05,0.0], "up":[0.0,0.05], "left":[0.05, 0.0]}
        self.dir_time = {"down":2000, "right":4000, "up":2000, "left":4000}
        self.curr_dir = "down"
        self.loops = 0
        self.cycles = 0

        #Arm params
        self.frame_id = "arm_gripper"
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_pitch_joint', 
                  'elbow_roll_joint', 'wrist_pitch_joint', 'wrist_roll_joint']

    def make_client(self, srv_type, name):
        """ Create a client to a service.

        Parameters
        ----------
        srv_type : Service Type
            The service type defined in the .srv file.
        name : string
            The name of the service call.
        """
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client
    
    def switch_controller(self, servo=False, sim=False):
        """Switch the ros2 control controllers.

        Parameters
        ----------
            servo : Bool
                Indicator for servo or trajectory controller
            sim : Bool
                Indicator for in sim mode (note: doesn't do anything). 
        
        """
        # Switches controller from forward position controller to joint_trajectory controller
        self.request = SwitchController.Request()
        if servo:
            if not sim:
                self.request.activate_controllers = ["rover_arm_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            else:
                self.request.activate_controllers = ["rover_arm_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            self.servo = True
        else:
            if not sim:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_controller"]
            else:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_controller"]
            self.servo = False
        self.request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        self.request.strictness = SwitchController.Request.BEST_EFFORT  # Use STRICT or BEST_EFFORT

        self.future = self.controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def configure_servo(self, frame):
        """Change the moveit servo planning frame.

        Parameters
        ----------
        frame : string
            The name of the planning frame.
        """
        # Changes planning frame of the servo node
        #arguments: "base_link" for base frame, "tool0" for tool frame
        req = SetParameters.Request()

        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=new_param_value)]
        self.future = self.configure_servo_cli.call_async(req)

    def joint_states_callback(self, msg):
        self.latest_joint_state = msg

    def move_to_joint_positions(self, joint_pose):
        """"Move arm to a set of joint angles.

        Parameters
        ----------
        joint_pose : Float Array
            The joint position in [rad] corresponding to each joint from base to wrist.
        """

        self.move_success = False

        # Wait until we have received joint states
        while self.latest_joint_state is None:
            return
        
        # Create a motion planning request
        motion_request = MotionPlanRequest()
        motion_request.workspace_parameters.header.frame_id = "base_link"
        motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Set the planning group
        motion_request.group_name = "rover_arm"
        
        # Set start state to the current state
        motion_request.start_state = RobotState()
        motion_request.start_state.joint_state.header = self.latest_joint_state.header
        motion_request.start_state.joint_state.name = self.latest_joint_state.name
        motion_request.start_state.joint_state.position = self.latest_joint_state.position
        motion_request.start_state.joint_state.velocity = self.latest_joint_state.velocity
        
        # Set the goal constraints based on the desired joint positions
        motion_request.goal_constraints = [Constraints()]
        
        for i, joint_name in enumerate(self.joint_names):
            # Create a joint constraint for each joint
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_pose[i]
            joint_constraint.tolerance_above = 0.01  # radians
            joint_constraint.tolerance_below = 0.01  # radians
            joint_constraint.weight = 1.0
            
            # Add the joint constraint to the goal constraints
            motion_request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Set planning parameters
        motion_request.max_velocity_scaling_factor = 0.5
        motion_request.max_acceleration_scaling_factor = 0.5
        motion_request.allowed_planning_time = 5.0
        motion_request.num_planning_attempts = 10
        
        # Create planning options
        planning_options = PlanningOptions()
        planning_options.plan_only = False  # Set to True if you only want to plan without execution
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 5
        #planning_options.replan_delay = Duration(sec=2, nanosec=0)
        
        # Create the goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_request
        goal_msg.planning_options = planning_options
        
        # Send the goal
        self.get_logger().info(f'Sending goal to move joints {self.joint_names} to positions {joint_pose}')
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        self.sent_goal = True
        
        # Add callbacks for goal response and result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return 
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return
            
        self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status        
        if status == 4:  # Succeeded
            self.get_logger().info('Motion execution succeeded!')
            self.move_success = True

        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            self.move_success = False

    

    def timer_callback(self): 
        #step 1 move to scan position
        if self.state == "start":
            #self.get_logger().info(f"{self.state}")
            if self.servo:
                self.get_logger().info("switch")
                self.switch_controller(servo=False)
                # time.sleep(0.1)
            start_scan_pose = [
                -0.5284632853455362, 
                -1.1027042707701618,  
                -0.7785432468899975, 
                -0.005995412834011991,
                -1.2662599658015512, 
                0.531499254831238
            ]

            #start_scan_pose = [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                self.get_logger().info("goal")
                self.move_to_joint_positions(start_scan_pose)
            if self.move_success:
                self.get_logger().info("goto scan")
                self.state = "scan"
                self.sent_goal = False



        #step 2 scan workspace
        if self.state == "scan":
            if not self.servo:
                self.switch_controller(servo=True)
                # time.sleep(0.1)

            #Draw a square
            self.set_direction()

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.twist = self.construct_twist()

            self.publisher_.publish(msg)

            if self.cycles > 1:
                self.state = "move_to_input"
                self.cycles = 0


        #step 3 move to get user input position
        if self.state == "move_to_input":
            if self.servo:
                self.switch_controller(servo=False)
                # time.sleep(0.1)
            input_pos =  [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.state = "get_pick_input"
                self.sent_goal = False


        #step 4 get pick input

        #step 5 move to above object

        #step 6 approach and pick

        #step 7 move to user input position

        #step 8 get place input

        #step 9 move to above place

        #step 10 place object

        #step 11 Return to user input

    def set_direction(self):

        if self.loops == self.dir_time[self.curr_dir]:
            self.loops = 0
            prev_dir = self.curr_dir
            self.curr_dir = self.sequence[self.curr_dir]
            if self.curr_dir == "down" and prev_dir == "left":
                self.cycles += 1
            self.get_logger().info("Going {}!".format(self.curr_dir))

        self.loops = self.loops + 1

    def construct_twist(self):
        
        twist = Twist()
        vals = self.dir_def[self.curr_dir]

        twist.linear.x = vals[0]
        twist.linear.y = vals[1]

        return twist


def main(args=None):
    rclpy.init(args=args)

    pick_and_place = SquareMakingController()

    # while rclpy.ok():
    #     rclpy.spin_once(pick_and_place)
    rclpy.spin(pick_and_place)

    # executor = MultiThreadedExecutor()
    # executor.add_node(pick_and_place)
    # executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pick_and_place.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()