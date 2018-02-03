#!/usr/bin/env python
import rospy
import math
import numpy
import time
from geometry_msgs.msg import Point,Pose
from foundations_hw3.srv import *
    
def callbackp2a(data):
    global start
    global goal
    global num
    global step
    global d
    step=0.02
    d=0.01
    costfun = rospy.ServiceProxy('compute_cost', ComputeCost)
    rospy.wait_for_service('compute_cost')
    start=data.start
    goal=data.goal
    num=data.num_waypoints
    deltax=(goal.x-start.x)/(num-1)
    deltay=(goal.y-start.y)/(num-1)
    PA=[]
    for i in range(0,num):
        p=Point(start.x+i*deltax,start.y+i*deltay,0.0)
        PA.append(p)
    for j in range(1,num-1):
        xi=PA[j].x
        yi=PA[j].y
        tagx=0
        while True:
            xii=PA[j].x=xi
            cost1=costfun(PA).cost
            print('x-cost: '+str(cost1))
            PA[j].x=xi+d
            cost2=costfun(PA).cost
            xi=xi-step*(cost2-cost1)/d
            tagx+=1
            if abs(xii-xi)<0.01 or tagx>500:
                break
        tagy=0
        while True:
            yii=PA[j].y=yi
            cost1=costfun(PA).cost
            print('y-cost: '+str(cost1))
            PA[j].y=yi+d
            cost2=costfun(PA).cost
            yi=yi-step*(cost2-cost1)/d
            tagy+=1
            if abs(yii-yi)<0.01 or tagy>500:
                break
    #rospy.loginfo(point)
    #print('result: ')
    #print(PA)
    return FindPathResponse(PA)
    

def p2a_server():
    rospy.init_node('p2a_server')

    s=rospy.Service('/p2a', FindPath, callbackp2a)
    
    rospy.spin()
    
if __name__ == '__main__':
    p2a_server()