#!/usr/bin/env python3

import sys
import rospy
from master.srv import AddTwoInts
from master.srv import AddTwoIntsRequest
from master.srv import AddTwoIntsResponse

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException:
        print ("Service call failed")

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print (usage())
        sys.exit(1)
    print ("Requesting %s+%s"%(x, y))
    s = add_two_ints_client(x, y)
    print ("%s + %s = %s"%(x, y, s))