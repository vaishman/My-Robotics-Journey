#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np



class OdomScanSubscriber(Node):
    def __init__(self):
        super().__init__('odom_scan_subscriber')
        
        # Create subscriber for odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create subscriber for laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Odom and Scan Subscriber Node Started')
        self.get_logger().info('Subscribing to /odom and /scan topics')
    
    def odom_callback(self, msg):
        """Callback function for odometry messages"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        lin_vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular

        p1= np.array((0,0,0))

    

        p2=np.array(pos)
        

        temp = p1-p2

        

        euclidian_dist =  np.sqrt(np.dot(temp.T, temp))

        if euclidian_dist >= 5.0:
            self.get_logger().info(
            f'\n--- ODOMETRY ---\n'
            f'Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n'
            f'Orientation: x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}\n'
            f'Linear Vel: x={lin_vel.x:.3f} \n'
            f'Angular Vel: z={ang_vel.z:.3f}'
            )
            p1 = p2
            p2 = np.array(pos)
        
        
        
        
    
    def scan_callback(self, msg):
        """Callback function for laser scan messages"""
        # Get min and max ranges from the scan
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            avg_range = sum(valid_ranges) / len(valid_ranges)
        else:
            min_range = max_range = avg_range = 0.0
        
        self.get_logger().info(
            f'\n--- LASER SCAN ---\n'
            f'Angle: min={msg.angle_min:.3f}, max={msg.angle_max:.3f}, increment={msg.angle_increment:.3f}\n'
            f'Range: min={msg.range_min:.3f}, max={msg.range_max:.3f}\n'
            f'Number of readings: {len(msg.ranges)}\n'
            f'Closest obstacle: {min_range:.3f}m\n'
            f'Farthest reading: {max_range:.3f}m\n'
            f'Average range: {avg_range:.3f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomScanSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()