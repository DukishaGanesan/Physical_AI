#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TempWarning(Node):
    def __init__(self):
         super().__init__('temp_warning')
         self.subscription = self.create_subscription(
             Float32,
            'temperature',
             self.callback,
             10)

    def callback(self, msg):
         if msg.data > 30.0:
             self.get_logger().warning(f'High temperature detected: {msg.data:.2f} °C!')
         else:
             self.get_logger().info(f'Temperature is normal: {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    node = TempWarning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()
