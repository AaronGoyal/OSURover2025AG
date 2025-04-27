import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class JoyToVelocityNode(Node):
    def __init__(self):
        super().__init__('joy_to_velocity')

        
        # Publisher for left and right wheel velocities
        #----
        self.wheel_pub = self.create_publisher(JointState, '/joint_states', 10)
        #self.right_wheel_pub = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10.0)
        #----
        # Subscriber to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',  # Topic where joy messages are published
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning

        linear_velocity = msg.axes[1]  # Left joystick vertical axis (forward/backward)
        angular_velocity = msg.axes[0]  # Right joystick horizontal axis (turning)

        # You may want to scale these velocities to suit your needs (e.g., make them faster/slower)
        left_velocity = linear_velocity - angular_velocity
        right_velocity = linear_velocity + angular_velocity

        # Prepare message for left and right wheels
       
        drive_msg = JointState(name = ["front_left", "middle_left", "back_left", "front_right", "middle_right", "back_right"],
                               position = [0.0,0.0,0.0,0.0,0.0,0.0],
                               velocity = [left_velocity, left_velocity, left_velocity,right_velocity, right_velocity, right_velocity],
                               effort = [0.0,0.0,0.0,0.0,0.0,0.0])
        
        # Publish the velocities
        self.wheel_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
