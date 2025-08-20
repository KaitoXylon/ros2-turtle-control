import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Square(Node):
    def __init__(self):
        super().__init__('Control')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.9, self.squarei)


        self.side=0
        self.state="moving"

    def squarei(self):
        msg = Twist()

        if self.state =="moving":
            msg.linear.x = 3.0
            msg.angular.z= 0.0
            self.publisher_.publish(msg)
            self.state = "turning"
            self.get_logger().info("Moving")

        elif self.state =="turning":

            msg.linear.x= 0.0
            msg.angular.z=1.73
            self.publisher_.publish(msg)
            self.state = "moving"
            self.side+=1
            self.get_logger().info("Turning")

        if self.side >=40 :
            self.timer.cancel()
            final_msg = Twist()
            self.publisher_.publish(final_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Square()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()