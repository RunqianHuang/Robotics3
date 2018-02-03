#!/usr/bin/env python
import rospy
import math
import numpy
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Point,Pose,Twist,Quaternion
from foundations_hw3.srv import *

def callback(data):
    #rospy.loginfo(data)
    global pp
    global oo
    pp=data.position
    oo=data.orientation
    #rospy.loginfo(pp)

def callbackp2b(data):
    global path
    global pp
    global oo
    global d
    global v
    path=data.path
    end=path[len(path)-1]
    pp=Point(0.0,0.0,0.0)
    oo=Quaternion(0.0,0.0,0.0,0.0)
    d=0.05
    v0=5
    horizon = rospy.ServiceProxy('interpolate_path', InterpolatePath)
    rospy.wait_for_service('interpolate_path')
    close = rospy.ServiceProxy('closest_point_path', ClosestPointPath)
    rospy.wait_for_service('closest_point_path')
    rospy.wait_for_message('/vrep/youbot/base/pose', Pose)
    rospy.Subscriber('/vrep/youbot/base/pose', Pose, callback)
    pub=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(2)
    fd=((end.x-pp.x)*(end.x-pp.x)+(end.y-pp.y)*(end.y-pp.y))
    while fd>=0.2:
        #print(pp)
        message=Twist()
        closeresult=close(pp,path)
        closest_point=closeresult.closest_point
        path_position=closeresult.path_position
        dist_to_path=closeresult.dist_to_path
        #print(dist_to_path)
        h_point=horizon(path,Float32(path_position.data+d)).point
        theta=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        dx=h_point.x-pp.x
        dy=h_point.y-pp.y
        xh=dx*math.cos(theta)+dy*math.sin(theta)
        yh=dx*math.sin(theta)+dy*math.cos(theta)
        fd=((end.x-pp.x)*(end.x-pp.x)+(end.y-pp.y)*(end.y-pp.y))
        #print(end)
        print(fd)
        v=v0
        if yh>=0:
            K=2*yh/(xh*xh+yh*yh)
            w=v*K-30
        else:
            K=-2*yh/(xh*xh+yh*yh)
            w=v*K+30
        #if dist_to_path<=0.1:
        #    w=0
        #print(v)
        #print(w)
        message.linear.x=0
        message.linear.y=v
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=w
        
        pub.publish(message)
        rate.sleep()   
    #rospy.spin()
    return FollowPathResponse()
    

def p2b_server():
    rospy.init_node('p2b_server')

    s=rospy.Service('/p2b', FollowPath, callbackp2b)
    
    rospy.spin()
    
if __name__ == '__main__':
    p2b_server()