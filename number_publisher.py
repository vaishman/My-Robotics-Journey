#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

if __name__== '__main__':
    rospy.init_node('number_publisher')
    rospy.set_param('/pub_freq', 1)
    rospy.set_param('/pub_data', 10)
    pub = rospy.Publisher('/number',Int64, queue_size=10 )
    
    pub_freq = rospy.get_param('/pub_freq')

    rate = rospy.Rate(pub_freq)

    pub_data = rospy.get_param('/pub_data')

   

    while not rospy.is_shutdown():
        msg = Int64()
        msg.data = pub_data
        pub.publish(msg)
        rate.sleep()