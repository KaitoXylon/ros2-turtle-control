import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 
import sys

class Input(Node):
    def __init__(self):
        super().__init__('target_input')
        self.publisher_ = self.create_publisher(Point, 'target_location', 10)
        self.timer =self.create_timer(0.9, self.input)

       
    def input(self):
        
        self.get_logger().info("Enter lat")
        x_input = input("lat: ")
        self.get_logger().info("Enter lon")
        y_input = input("lon: ")

        target_x=float(x_input)
        target_y=float(y_input)

        target_msg = Point()
        target_msg.x = target_x
        target_msg.y = target_y
        target_msg.z = 0.0

        self.publisher_.publish(target_msg)
        self.get_logger().info(f"Published new target: ({target_x}, {target_y})")


def main(args=None):
    rclpy.init(args=args)
    target_publisher_node = Input()
    rclpy.spin(target_publisher_node)
    target_publisher_node.destroy()
    rclpy.shutdown()
if __name__ == '__main__':
    main()