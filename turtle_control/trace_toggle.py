

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.srv import SetPen

class TraceToggle(Node):

    def __init__(self):
        super().__init__('trace_toggle')
        self.srv = self.create_service(SetBool, '/toggle_trace', self.handle_toggle)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')

        self.pen_on = True 

    def handle_toggle(self, request, response):
        self.pen_on = request.data

        pen_req = SetPen.Request()
        pen_req.r = 255
        pen_req.g = 0
        pen_req.b = 0
        pen_req.width = 2
        pen_req.off = int(not self.pen_on)

        self.pen_client.call_async(pen_req)

        response.success = True
        response.message = f"Pen {'ON' if self.pen_on else 'OFF'}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TraceToggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
