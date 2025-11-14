import rclpy
from rclpy.node import Node
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal with order {goal_handle.request.order}')

        # Simple Fibonacci calculation
        sequence = [0, 1]
        for i in range(2, goal_handle.request.order):
            sequence.append(sequence[i-1] + sequence[i-2])

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence[:goal_handle.request.order]
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
