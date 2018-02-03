#!/usr/bin/env python
import rospy
import math
import numpy
import time
from geometry_msgs.msg import Point,Pose
from foundations_hw3.srv import *

def test_p2a():
    rospy.init_node('testp2a')
    test = rospy.ServiceProxy('p2a', FindPath)
    
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rospy.wait_for_service('p2a')

        path = test(Point(0.0,0.0,0.0),Point(2.0,2.0,2.0),3).path
        print('testresult: ')
        print(path)

        rate.sleep()   
    rospy.spin()
    
if __name__ == '__main__':
    test_p2a()