#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0
        self.number_count_pub = self.create_publisher(Int64, 'number_count', 10)
        self.number_subscriber = self.create_subscription(Int64, 'number', self.callback_number, 10)
        self.reset_counter_srv = self.create_service(SetBool, 'reset_counter', self. callback_reset_counter )
        self.get_logger().info('Number counter has been started.')



    def  callback_number(self, msg):
        self.counter += msg.data 
        #self.get_logger().info(str(self.counter))
        new_msg= Int64()
        new_msg.data = self.counter
        self.number_count_pub.publish(new_msg)        
    
    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        if request.data == True:
            self.counter = 0
            response.success = True
            response.message = 'counter has been reset'

        else:
            response.success = False
            response.message = 'Counter not reset'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node =NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()