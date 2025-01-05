#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

from enum import Enum

import smach_ros
from time import sleep
import time
import math
#import pigpio
import subprocess
from MasterConfig import MASTER_SMachineStates_ten #as Mch
from MasterConfig import MASTER_MasterStates_ten #as Ma
from MasterConfig import MASTER_NavigatorStates_ten #as Nv

#from HafnaouiSMach import * 
from bob import * 

def ColorRequest_Callback (msg):
    if (msg.data == 1):
        print ('SEARCHING PINK/YELLOW')
        # pink/yellow stack serach here 
    elif (msg.data == 3): 
        print ('SEARCHING Brown')
        # intergrate brown stack serach here 

def XY_StackRequest_Callback (msg):

    if (msg.data):
        # send the generated XY here  
        data = Float32MultiArray()
        data.data = [20,30,1] # 1 refers to pink, 2 refers to yellow and 3 refers to brown 
        Target.publish(data)
    
def main():

    global Target 

    rospy.init_node('strategy', anonymous=True)

    Target = rospy.Publisher("NextTarget", Float32MultiArray, queue_size=10) 

    rospy.Subscriber("ColorRequest", Int8, ColorRequest_Callback)
    rospy.Subscriber("XY_StackRequest", Bool, XY_StackRequest_Callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    main()
