#!/usr/bin/env python
import rospy
import math
import numpy
from cvxopt import matrix,solvers
from geometry_msgs.msg import Point
from foundations_hw3.srv import *
  
def p1a_server():
    rospy.init_node('p1a_server')
    rospy.wait_for_service('/vrep/block1/pos/get')  
    rospy.wait_for_service('/vrep/block2/pos/get')  
    rospy.wait_for_service('/vrep/block3/pos/get')   
    rospy.wait_for_service('/vrep/block4/pos/get')    
    rospy.wait_for_service('/vrep/block5/pos/get')
    rospy.wait_for_service('vrep/bucket/pos/set')
    
    P1 = rospy.ServiceProxy('/vrep/block1/pos/get', GetPosition)
    P2 = rospy.ServiceProxy('/vrep/block2/pos/get', GetPosition)
    P3 = rospy.ServiceProxy('/vrep/block3/pos/get', GetPosition)
    P4 = rospy.ServiceProxy('/vrep/block4/pos/get', GetPosition)
    P5 = rospy.ServiceProxy('/vrep/block5/pos/get', GetPosition)
    P0 = rospy.ServiceProxy('vrep/bucket/pos/set', SetPosition)
    p1=P1().pos
    p2=P2().pos
    p3=P3().pos
    p4=P4().pos
    p5=P5().pos
    #rospy.loginfo(p1)
    #rospy.loginfo(p2)
    #rospy.loginfo(p3)
    #rospy.loginfo(p4)
    #rospy.loginfo(p5)
    maxx=max(p1.x,p2.x,p3.x,p4.x,p5.x)
    minx=min(p1.x,p2.x,p3.x,p4.x,p5.x)
    maxy=max(p1.y,p2.y,p3.y,p4.y,p5.y)
    miny=min(p1.y,p2.y,p3.y,p4.y,p5.y)
    P=matrix([[5.0,0.0],[0.0,5.0]])
    q=matrix([-2.0*(p1.x+p2.x+p3.x+p4.x+p5.x),-2.0*(p1.y+p2.y+p3.y+p4.y+p5.y)])
    G=matrix([[-1.0,1.0,0.0,0.0],[0.0,0.0,-1.0,1.0]])
    h=matrix([-minx,maxx,-miny,maxy])
    result=solvers.qp(P,q,G,h)
    resx=result['x'][0]
    resy=result['x'][1]
    res=Point(resx,resy,0.0)
    #rospy.loginfo(res)
    P0(res)
 
    rospy.spin()
    
if __name__ == '__main__':
    p1a_server()