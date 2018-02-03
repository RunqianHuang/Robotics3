#!/usr/bin/env python
import rospy
import math
import numpy
import time
from std_msgs.msg import *
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point,Pose,Twist,Quaternion
from foundations_hw3.srv import *

def callback(data):
    #rospy.loginfo(data)
    global pp
    global oo
    pp=data.position
    oo=data.orientation
    #rospy.loginfo(pp)

def p3():
    rospy.init_node('p3')
    rospy.loginfo('node_start')
    global pp
    global oo
    pp=Point(0.01,0.0,0.0)
    oo=Quaternion(0.0,0.0,0.0,0.0)
    rospy.Subscriber('/vrep/youbot/base/pose', Pose, callback)
    rospy.wait_for_service('/vrep/block1/pos/get')  
    rospy.wait_for_service('/vrep/block2/pos/get')  
    rospy.wait_for_service('/vrep/block3/pos/get')   
    rospy.wait_for_service('/vrep/block4/pos/get')    
    rospy.wait_for_service('/vrep/block5/pos/get')
    rospy.wait_for_service('vrep/bucket/pos/get')
    P1 = rospy.ServiceProxy('/vrep/block1/pos/get', GetPosition)
    P2 = rospy.ServiceProxy('/vrep/block2/pos/get', GetPosition)
    P3 = rospy.ServiceProxy('/vrep/block3/pos/get', GetPosition)
    P4 = rospy.ServiceProxy('/vrep/block4/pos/get', GetPosition)
    P5 = rospy.ServiceProxy('/vrep/block5/pos/get', GetPosition)
    P0 = rospy.ServiceProxy('vrep/bucket/pos/get', GetPosition)
    p1=P1().pos
    p2=P2().pos
    p3=P3().pos
    p4=P4().pos
    p5=P5().pos
    o1=P1().orientation
    o2=P2().orientation
    o3=P3().orientation
    o4=P4().orientation
    o5=P5().orientation
    p0=P0().pos
    findpath = rospy.ServiceProxy('p2a', FindPath)
    rospy.wait_for_service('p2a')
    move = rospy.ServiceProxy('p2b', FollowPath)
    rospy.wait_for_service('p2b')
    reach = rospy.ServiceProxy('vrep/youbot/arm/reach', ReachPos)
    rospy.wait_for_service('vrep/youbot/arm/reach')
    close = rospy.ServiceProxy('vrep/youbot/gripper/grip',SetBool)
    rospy.wait_for_service('vrep/youbot/gripper/grip')
    pubp1=rospy.Publisher('/vrep/youbot/arm/joint1/angle', Float64, queue_size=10)
    pubp2=rospy.Publisher('/vrep/youbot/arm/joint2/angle', Float64, queue_size=10)
    pubp3=rospy.Publisher('/vrep/youbot/arm/joint3/angle', Float64, queue_size=10)
    pubp4=rospy.Publisher('/vrep/youbot/arm/joint4/angle', Float64, queue_size=10)
    pubp5=rospy.Publisher('/vrep/youbot/arm/joint5/angle', Float64, queue_size=10)
    
    #rospy.wait_for_message('/vrep/youbot/base/pose', Pose)
    rospy.loginfo('start_findpath1')
    path1 = findpath(pp,p1,5).path
    rospy.loginfo('path1_finded')
    rospy.loginfo(path1)
    move(path1)
    rospy.loginfo('reached1')
    pub1=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while True:
        rospy.loginfo('start_turn1')
        theta1=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        theta2=math.atan2(2*(o1.y*o1.w-o1.x*o1.z),1-2*(o1.z*o1.z+o1.y*o1.y))
        if abs(theta1-theta2)<19*math.pi/36 and abs(theta1-theta2)>17*math.pi/36:
            break
        rospy.loginfo(math.acos(2*oo*o1-1))
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=math.pi
        
        pub1.publish(message)
        rate.sleep()   
    rate = rospy.Rate(10)
    while True:
        ifreach=reach(p1)
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        
        pub1.publish(message)
        rate.sleep()   
        if ifreach==True:
            close(True)
            break
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    time.sleep(2)
    path10 = findpath(pp,p0,5).path
    move(path10)
    ifreach=reach(p0)
    if ifreach==True:
        close(False)
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    
    rospy.loginfo('start_findpath2')
    path2 = findpath(pp,p2,5).path
    rospy.loginfo('path2_finded')
    rospy.loginfo(path2)
    move(path2)
    rospy.loginfo('reached2')
    pub2=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while True:
        rospy.loginfo('start_turn2')
        theta1=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        theta2=math.atan2(2*(o1.y*o1.w-o1.x*o1.z),1-2*(o1.z*o1.z+o1.y*o1.y))
        if abs(theta1-theta2)<19*math.pi/36 and abs(theta1-theta2)>17*math.pi/36:
            break
        rospy.loginfo(math.acos(2*oo*o1-1))
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=math.pi
        
        pub2.publish(message)
        rate.sleep()   
    rate = rospy.Rate(10)
    while True:
        ifreach=reach(p2)
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        
        pub2.publish(message)
        rate.sleep()   
        if ifreach==True:
            close(True)
            break
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    time.sleep(2)
    path20 = findpath(pp,p0,5).path
    move(path20)
    ifreach=reach(p0)
    if ifreach==True:
        close(False)
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    
    rospy.loginfo('start_findpath3')
    path3 = findpath(pp,p3,5).path
    rospy.loginfo('path3_finded')
    rospy.loginfo(path3)
    move(path3)
    rospy.loginfo('reached3')
    pub3=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while True:
        rospy.loginfo('start_turn3')
        theta1=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        theta2=math.atan2(2*(o1.y*o1.w-o1.x*o1.z),1-2*(o1.z*o1.z+o1.y*o1.y))
        if abs(theta1-theta2)<19*math.pi/36 and abs(theta1-theta2)>17*math.pi/36:
            break
        rospy.loginfo(math.acos(2*oo*o1-1))
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=math.pi
        
        pub3.publish(message)
        rate.sleep()   
    rate = rospy.Rate(10)
    while True:
        ifreach=reach(p3)
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        
        pub3.publish(message)
        rate.sleep()   
        if ifreach==True:
            close(True)
            break
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    time.sleep(2)
    path30 = findpath(pp,p0,5).path
    move(path30)
    ifreach=reach(p0)
    if ifreach==True:
        close(False)
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    
    rospy.loginfo('start_findpath4')
    path4 = findpath(pp,p4,5).path
    rospy.loginfo('path4_finded')
    rospy.loginfo(path4)
    move(path4)
    rospy.loginfo('reached4')
    pub4=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while True:
        rospy.loginfo('start_turn4')
        theta1=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        theta2=math.atan2(2*(o1.y*o1.w-o1.x*o1.z),1-2*(o1.z*o1.z+o1.y*o1.y))
        if abs(theta1-theta2)<19*math.pi/36 and abs(theta1-theta2)>17*math.pi/36:
            break
        rospy.loginfo(math.acos(2*oo*o1-1))
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=math.pi
        
        pub4.publish(message)
        rate.sleep()   
    rate = rospy.Rate(10)
    while True:
        ifreach=reach(p4)
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        
        pub4.publish(message)
        rate.sleep()   
        if ifreach==True:
            close(True)
            break
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    time.sleep(2)
    path40 = findpath(pp,p0,5).path
    move(path40)
    ifreach=reach(p0)
    if ifreach==True:
        close(False)
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    
    rospy.loginfo('start_findpath5')
    path5 = findpath(pp,p5,5).path
    rospy.loginfo('path5_finded')
    rospy.loginfo(path5)
    move(path5)
    rospy.loginfo('reached5')
    pub5=rospy.Publisher('vrep/youbot/base/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    while True:
        rospy.loginfo('start_turn5')
        theta1=math.atan2(2*(oo.y*oo.w-oo.x*oo.z),1-2*(oo.z*oo.z+oo.y*oo.y))
        theta2=math.atan2(2*(o1.y*o1.w-o1.x*o1.z),1-2*(o1.z*o1.z+o1.y*o1.y))
        if abs(theta1-theta2)<19*math.pi/36 and abs(theta1-theta2)>17*math.pi/36:
            break
        rospy.loginfo(math.acos(2*oo*o1-1))
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=math.pi
        
        pub5.publish(message)
        rate.sleep()   
    rate = rospy.Rate(10)
    while True:
        ifreach=reach(p5)
        message=Twist()
        message.linear.x=0
        message.linear.y=0
        message.linear.z=0
        message.angular.x=0
        message.angular.y=0
        message.angular.z=0
        
        pub5.publish(message)
        rate.sleep()   
        if ifreach==True:
            close(True)
            break
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    time.sleep(2)
    path50 = findpath(pp,p0,5).path
    move(path50)
    ifreach=reach(p0)
    if ifreach==True:
        close(False)
    pubp1.publish(1.57)
    pubp2.publish(0)
    pubp3.publish(0)
    pubp4.publish(0)
    pubp5.publish(0)
    
    rospy.spin()
    
if __name__ == '__main__':
    p3()