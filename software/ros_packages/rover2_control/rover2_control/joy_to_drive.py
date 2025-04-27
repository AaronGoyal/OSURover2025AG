import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyToVelocityNode(Node):
    def __init__(self):
        super().__init__('joy_to_velocity')

        # Publisher for left and right wheel velocities
        self.left_wheel_pub = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.right_wheel_pub = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)

        # Subscriber to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy_drive',  # Topic where joy messages are published
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
        left_msg = Float64MultiArray(data=[left_velocity, left_velocity, left_velocity])
        right_msg = Float64MultiArray(data=[right_velocity, right_velocity, right_velocity])

        # Publish the velocities
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
