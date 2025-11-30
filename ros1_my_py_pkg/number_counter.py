#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool 


counter = 0
freq = 0
def callback_number(msg):
   
   global counter, freq
   counter += msg.data
   freq += 1
   new_msg = Int64()
   new_msg.data = counter
   pub_counter.publish(new_msg)
   rospy.loginfo(new_msg)

   freq_msg = Int64()
   freq_msg.data = freq
   pub_freq.publish(freq_msg)
   rospy.loginfo(freq_msg)

def srv_callback(req):
    global counter, freq

    
    if req.data == 0:
        counter = 0
        freq = 0
        return  True, 'counter reset to 0'
    else:
        return False, 'no reset performed'

    
    

if __name__== '__main__':
    rospy.init_node('number_counter')

    service = rospy.Service('/reset_number_count', SetBool, srv_callback)

    pub_counter = rospy.Publisher('/number_counter', Int64, queue_size= 10)

    pub_freq = rospy.Publisher('/freq', Int64, queue_size= 10)

    sub = rospy.Subscriber('/number', Int64, callback_number )

    rospy.spin()
