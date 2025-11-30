#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):
    def __init__ (self):
        super().__init__('add_two_ints_client')

    def call_add_two_ints(self, a , b):
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)

        future.add_done_callback(partial(self.callback_call_add_two_ints, request= request))

    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(str(request.a)+ ' + '+ str(request.b)+ '='+ str( response.sum))


def main(args=None):
    rclpy.init()
    node = AddTwoIntsClient()
    node.call_add_two_ints(2, 7)
    node.call_add_two_ints(5, 3)
    node.call_add_two_ints(1, 7)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()