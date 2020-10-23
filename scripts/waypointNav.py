#!/usr/bin/env/python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import random
from numpy import linspace
import tf
from tf import transformations
goalPublisher= rospy.Publisher("goal_pose",Pose,queue_size=1)

waypoints=[]
curr_x= None
curr_y=None
currPos= Pose()
goalPos= Pose()
reachable=None

def waypointSplitter():
    global currPos,goalPos,waypoints
    # waypoints.append(goalPos)
    temp=currPos
    n= abs(goalPos.position.x-currPos.position.x)
    slope= goalPos.position.y-currPos.position.y/goalPos.position.x-currPos.position.x
    xPoints= linspace(currPos,goalPos,n)
    for i in xPoints:
        temp=Pose()
        temp.position.x= i
        temp.position.y=(i*slope)
        waypoints.append(temp)
    # i=0.3 if currPos.position.x>=goalPos.position.x else -0.3
    # for temp.position.x in range(currPos.position.x,goalPos.position.x): 
    #     temp.position.x=currPos.position.x+i
    # i=temp.position.y/temp.position.x if currPos.position.y>=goalPos.position.y else -temp.position.y/temp.position.x
    # for temp.position.y in range(currPos.position.y,goalPos.position.y): 
    #     temp.position.y=currPos.position.y+i
    # waypoints.append(temp)    

def unreachableCheck(msg):
    global reachable
    reachable=msg.data

def goalStack():
    global waypoints
    while len(waypoints):
        goal= waypoints[-1] 
        if not reachable:
            temp=Pose()
            temp.position.x= random.randrange(goal.position.x-0.3,goal.position.x+0.3,0.1)
            temp.position.y= random.randrange(goal.position.y-0.3,goal.position.y+0.3,0.1)
            waypoints.pop()
            waypoints.append(temp)
        else:
            goalPublisher.publish(goal)
            rospy.loginfo("Published {} {}".format(goal.position.x,goal.position.y))
            waypoints.pop()
    # goalGen() ??

def goalGen():
    global currPos,goalPos
    goalPos.position.x= random.randrange(currPos.position.x-3,currPos.position.x+3,1)
    goalPos.position.y= random.randrange(currPos.position.y-3,currPos.position.y+3,1)
    waypoints.append(goalPos)
    waypointSplitter()


def listener():
    global currPos
    rospy.init_node('ria_wayPointPublisher',anonymous=True)
    rospy.Subscriber('/ria_nav/unreachableCheck',Bool,unreachableCheck)
    listener= tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (lin,rot)= listener.lookupTransform('/odom','/camera_link',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    currPos.position.x= lin[0]
    currPos.position.y= lin[1]
    goalGen()
    goatlStack()



if __name__=='__main__':
    listener()
