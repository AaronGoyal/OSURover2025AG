import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from example_interfaces.srv import Trigger
from example_interfaces.action import Fibonacci

class FibonacciService(Node):
    def __init__(self):
        super().__init__('fibonacci_service')
        self.srv = self.create_service(Trigger, 'trigger_fibonacci', self.handle_service)
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')

        # Keep track of pending responses
        self.pending_responses = {}

    def handle_service(self, request, response):
        self.get_logger().info('Service received request, sending action goal...')

        # Wait for the action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            response.success = False
            response.message = 'Action server not available'
            return response

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        # Send goal asynchronously
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f, resp=response: self.goal_response_callback(f, resp))

        # Return immediately; the callback will set the response
        return response

    def goal_response_callback(self, future, service_response):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            service_response.success = False
            service_response.message = 'Goal rejected by action server'
            return

        self.get_logger().info('Goal accepted, waiting for result...')

        # Attach callback for when the action result is ready
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, service_response))

    def get_result_callback(self, future, service_response):
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result.sequence}')
        service_response.success = True
        service_response.message = f'Action completed: {result.sequence}'

        # NOTE: If using a real service client, you'd now respond to it
        # Using ROS 2 Python `Trigger` service, the response object is returned immediately,
        # so for real async responses, you'd want an action or custom service type that can be fulfilled later.

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
