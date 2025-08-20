import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
import math


class autoTurtle(Node):
    def __init__(self):
        super().__init__('AutoTurtle')
        self.target_x = None
        self.target_y = None

        self.velo_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subcriber = self.create_subscription(Pose, '/turtle1/pose',self.pose_callback,10)

        self.target_subscriber_ = self.create_subscription(Point, 'target_location',self.target_callback, 10)
        self.current_pose = None
        self.linear_velo_kp = 1.5
        self.angular_velo_kp = 6.0
        self.tolerance = 0.1
        self.timer = self.create_timer(0.1, self.automove)

    def pose_callback(self, msg):

        self.current_pose = msg
        self.get_logger().info(f"Pose : X: {msg.x:.2f},y: {msg.y:.2f},theta:{msg.theta:.2f}")

    def target_callback(self,msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info(f'Received new target: ({self.target_x:.2f}, {self.target_y:.2f})')

    def automove(self):
        if self.current_pose is None or self.target_x is None:
            
            self.get_logger().info('Waiting for pose and/or target data...')
            return
        
        distance = math.sqrt((self.target_x - self.current_pose.x)**2 + (self.target_y - self.current_pose.y)**2)

        if distance < self.tolerance:
            self.publish_velo(0.0,0.0)
            self.get_logger().info(f'Goal rreached')
            self.target_x = None
            self.target_y = None
            return
        
        target_angle = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)
        angular_error = target_angle - self.current_pose.theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        linear_velo = self.linear_velo_kp * distance
        angular_velo = self.angular_velo_kp* angular_error

        self.publish_velo(linear_velo, angular_velo)
    
    def publish_velo(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x= linear
        twist_msg.angular.z = angular
        self.velo_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = autoTurtle()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()