import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
import json

class NavInput(Node):
    def __init__(self):
        super().__init__('navigation_input_node')
        self.publisher_ = self.create_publisher(String, 'navigation_coordinates', 10)
        self.timer = self.create_timer(2.0, self.publisher_coordinates)

    def publisher_coordinates(self):
        self.get_logger().info("Enter lat lon:")

        coords = []
        for i in range(2):
            lat = float(input(f"Latitude {i+1}: "))
            lon = float(input(f"Longitude {i+1}: "))
            coords.append({'lat': lat, 'lon': lon})


        msg = String()
        msg.data = json.dumps(coords)
        self.publisher_.publish(msg)
        self.get_logger().info("Coords published.")
        self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)  
    node = NavInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    