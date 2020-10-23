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
    waypoints.append(goalPos)
    temp=currPos
    # i=0.3 if currPos.position.x>=goalPos.position.x else -0.3
    # for temp.position.x in range(currPos.position.x,goalPos.position.x): 
    #     temp.position.x=currPos.position.x+i
    # i=temp.position.y/temp.position.x if currPos.position.y>=goalPos.position.y else -temp.position.y/temp.position.x
    # for temp.position.y in range(currPos.position.y,goalPos.position.y): 
    #     temp.position.y=currPos.position.y+i
    waypoints.append(temp)    

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
    #goalGen()

# def goalGen():
#     currPos.position.x=curr_x
#     currPos.position.y=curr_y
#     goalPos.position.x= random.randrange(currPos.position.x-3,currPos.position.x+3,1)
#     goalPos.position.y= random.randrange(currPos.position.y-3,currPos.position.y+3,1)
#     waypoints.append(goalPos)


def listener():
    rospy.init_node('ria_wayPointPublisher',anonymous=True)
    rospy.Subscriber('/ria_nav/unreachableCheck',Bool,unreachableCheck)
    listener= tf.TransformListener()

if __name__=='__main__':
    listener()
