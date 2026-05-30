import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from astra_msgs import msg


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
        self.sub = self.create_subscription(Twist, "/auto/cmd_vel", self.convert, 10)
        self.pub = self.create_publisher(msg.CoreControl, "/core/control", 10)
        self.get_logger().info("CmdVelBridge ready.")

    def convert(self, msg: Twist):
        # Convert Twist → DriveCommand
        # drive = DriveCommand()
        # drive.linear_velocity = msg.linear.x
        # drive.angular_velocity = msg.angular.z
        # self.pub.publish(drive)
        drive = msg.CoreControl

        self.get_logger().info(
            f"Bridged cmd_vel (x:{msg.linear.x:.2f}, z:{msg.angular.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
