#!/usr/bin/env python
import rospy
from robot_nav.msg import Marker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from math import pi, sin

_pi = pi*0.1
goal = [0,0]
marker_search = False
marker_detected = False
marker_aligned = False
goal_set = False

pub = rospy.Publisher("/ria/odom/local/goal",Twist, queue_size=1)

def detectCallback(msg):
    global marker_detected, marker_aligned, goal_set
    if not marker_search or not(msg.id == 2) or goal_set:
        return
    if  marker_detected and not marker_aligned:
        marker_aligned = msg.aligned
        return
    marker_detected = True
    rospy.wait_for_service('ria/odom/goal/stop')
    try:
        #set = SetBool()
        #set.data = True
        goal_stop = rospy.ServiceProxy('ria/odom/goal/stop', SetBool)
        resp = goal_stop(True)
        return resp
    except rospy.ServiceException as e:
        print("Service call falied: %s"%e)

    rospy.wait_for_service('ria/odom/goal/reset')
    try:
        goal_reset = rospy.ServiceProxy('ria/odom/goal/reset', Trigger)
        resp = goal_reset()
        return resp
    except rospy.ServiceException as e:
        print("Service call falied: %s"%e)
    
    rospy.wait_for_service('ria/odom/reset')
    try:
        odom_reset = rospy.ServiceProxy('ria/odom/reset', Trigger)
        resp = odom_reset()
        return resp
    except rospy.ServiceException as e:
        print("Service call falied: %s"%e)
    if marker_aligned:
        goal[0] = ((90 -abs(msg.theta))*_pi)/180
        if msg.theta <0:
            goal[0] = -1*goal[0]
        goal[1] = msg.distance*sin(abs(msg.theta))
        goal_set = True

def start_search(msg):
    global marker_search
    marker_search = msg.data
    response = SetBoolResponse()
    response.success = True
    rospy.loginfo("Searching Set: {}".format(marker_search))
    return response

def posCallback(msg):
    global marker_search, marker_detected, marker_aligned, goal_set
    s_ang = msg
    if marker_search and not marker_aligned and not goal_set:
        if  marker_detected and not marker_aligned:
            s_ang.angular.z += 1
        else:
            s_ang.angular.z += 2
        pub.publish(s_ang)
    elif marker_search and goal_set:
        s_ang.angular.z = goal[0]
        s_ang.linear.x = goal[1]

def listner():
    rospy.init_node('marker_search',anonymous=True)
    rospy.Subscriber('/ria/odom/marker',Marker, detectCallback)
    rospy.Subscriber('/ria/odom/local',Twist, posCallback)
    m_search = rospy.Service('ria/odom/marker/search',SetBool, start_search)
    rospy.spin()
    #while not rospy.is_shutdown():
    #    goal_publish()
if __name__=='__main__':
    listner()
