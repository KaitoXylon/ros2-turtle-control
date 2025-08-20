import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.9, self.draw_circle)


    def draw_circle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info("Turtle is drawing a circle...")



def main(args=None):
    rclpy.init(args=args)
    node = CircleDrawer()
    node.draw_circle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()