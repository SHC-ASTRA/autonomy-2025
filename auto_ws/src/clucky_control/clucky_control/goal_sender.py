import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ros2_interfaces_pkg import msg


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        # 1) Action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 2) Subscriber to external goals
        self._sub = self.create_subscription(
            # ExternalGoal, '/external_goal', self.goal_cb, 10
            PoseStamped, '/auto/external_goal', self.goal_cb, 10  # placeholder
        )

    def goal_cb(self, msg):
        self.get_logger().info(f"Received external goal: {msg}")
        goal_msg = NavigateToPose.Goal()
        # If your external msg already is a PoseStamped, you can assign directly:
        goal_msg.pose = msg
        # Otherwise map from your fields:
        # goal_msg.pose.header.frame_id = 'map'
        # goal_msg.pose.pose.position.x = msg.x
        # goal_msg.pose.pose.position.y = msg.y
        # goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().warn('Navigation failed.')

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
