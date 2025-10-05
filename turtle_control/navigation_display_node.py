import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
import json
import math



class NaviDisplay(Node):
    def __init__(self):
        super().__init__('navigation_display_node')
        self.subscription = self.create_subscription(String, 'navigation_coordinates', self.listener_callback, 10)


    def listener_callback(self, msg):
        coords = json.loads(msg.data)

        for i in range(len(coords)-1):
            lat1 , lon1 = coords[i]['lat'],coords[i]['lon']
            lat2 , lon2 = coords[i+1]['lat'] , coords[i+1]['lon']



            distance = self.haversine(lat1,lon1,lat2,lon2)

            bearing = self.bearing(lat1,lon1,lat2,lon2) 

            self.get_logger().info(f"From point {i+1} to {i+2}:")
            self.get_logger().info(f" Distanciaa: {distance:.2f}")
            self.get_logger().info(f" Bearing Angle: {bearing:.2f}°")

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 3396000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c


    def bearing(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)

        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(dlon)

        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        return (bearing + 360) % 360


def main(args=None):
    rclpy.init(args=args)
    node = NaviDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()