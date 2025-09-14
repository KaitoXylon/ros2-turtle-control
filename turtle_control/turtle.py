import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
import math


class turtle(Node):
    def __init__(self):
        super().__init__('Turtle')
        self.target_x = None
        self.target_y = None

        self.velo_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subcriber = self.create_subscription(Pose, '/turtle1/pose',self.pose_callback,10)

        self.target_subscriber_ = self.create_subscription(Point, '/target_location',self.target_callback, 10)
        self.timer = self.create_timer(0.1, self.turtle)

    def pose_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f"Pose : X: {msg.x:.2f},y: {msg.y:.2f},theta:{msg.theta:.2f}")

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info(f'Received new target: ({self.target_x:.2f}, {self.target_y:.2f})')

    def turtle(self):
        if self.current_pose is None or self.target_x is None:
            
            self.get_logger().info('waiting for target')
            return
          
        msg = Twist()
    
        bearing_angle = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)

        error = bearing_angle - self.current_pose.theta
    
        if error > 0.05 :
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            
        elif error< 0.05:
            msg.angular.z = -1.0
            msg.linear.x = 0.0
        else :
            msg.angular.z = 0.0

        distance = math.sqrt((self.target_x - self.current_pose.x)**2 +
                         (self.target_y - self.current_pose.y)**2)

        if distance >0.5:
            msg.linear.x = 2.0

        else:
            msg.linear.x = 0.0
            self.get_logger().info(f"Reached target! Distance: {distance:.2f}")

            msg.angular.z = 0.0
            msg.linear.x = 0.0

        self.velo_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = turtle()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        



