#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):    
    def __init__(self):
        super().__init__('number_publisher')
        self.declare_parameter('number', 2)
        self.declare_parameter('timer_period', 1.0)
        self.number = self.get_parameter('number').value
        self.timer_period_ = self.get_parameter('timer_period').value
        self.number_pub = self.create_publisher(Int64,'number', 10)
        self.number_timer = self.create_timer(self.timer_period_, self.publish_number)
        self.get_logger().info('Number publisher has been started')
    
    def publish_number(self):
        msg = Int64()
        msg.data= self.number
        self.number_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()




if __name__=='__main__':
    main()