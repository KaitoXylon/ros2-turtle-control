

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Figure8Driver(Node):

    def __init__(self):
        super().__init__('figure8_driver')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)  
        self.log_timer = self.create_timer(1.0, self.log_pose)  

        self.declare_parameter('pattern_speed', 1.0)
        self.pose = None
        self.time = 0.0

    def pose_callback(self, msg):
        self.pose = msg

    def log_pose(self):
        if self.pose:
            self.get_logger().info(f'Position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, θ={self.pose.theta:.2f}')

    def move_turtle(self):
        msg = Twist()
        speed = self.get_parameter('pattern_speed').get_parameter_value().double_value
        self.time += 0.1
        msg.linear.x = 2.5 * speed
        msg.angular.z = 2.5 * speed * math.sin(self.time)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
