#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from time import sleep

def nav_feedback():
        sleep(15)
        pub = rospy.Publisher('active', String, queue_size=10)
        rospy.init_node('nav_feedback', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown(): #condition this by the rising edge
            pub.publish('activate State Machine')
            rate.sleep()
   
if __name__ == '__main__':
   try:
       nav_feedback()
   except rospy.ROSInterruptException:
       pass
