#!/usr/bin/env python3
import rospy


if __name__ == '__main__':
    rospy.init_node('my_first_python_node')
    rospy.loginfo ("This node has been started")

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
         rospy.loginfo('Hi')
         rate.sleep()


