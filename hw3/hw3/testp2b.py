#!/usr/bin/env python
import rospy
import math
import numpy
import time
from geometry_msgs.msg import Point,Pose
from foundations_hw3.srv import *

def test_p2b():
    rospy.init_node('testp2b')
    #test1 = rospy.ServiceProxy('p2a', FindPath)
    #rospy.wait_for_service('p2a')
    test2 = rospy.ServiceProxy('p2b', FollowPath)
    rospy.wait_for_service('p2b')
    path = [Point(0.0,0.0,0.0),Point(0.2,0.0,0.0),Point(0.3,0.3,0.0),Point(0.4,0.35,0.0)]
    #print(path)
    test2(path)
    rospy.spin()
    
if __name__ == '__main__':
    test_p2b()