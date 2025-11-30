#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('robot_news_radio_transmitter')

    pub = rospy.Publisher("/robot_news_radio", String, queue_size= 10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hi from the news radio"
        pub.publish(msg)
        rate.sleep()

    rospy.logwarn("Node was stopped")
