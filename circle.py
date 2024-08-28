#!/usr/bin/env python3

import rospy
from my_robot_msgs.srv import ComputeDiskArea

def callback(req):
    area = 3.14*req.radius*req.radius
    return area

if __name__== '__main__':
    rospy.init_node('circle')

    service = rospy.Service('/area', ComputeDiskArea, callback )

    rospy.spin()


