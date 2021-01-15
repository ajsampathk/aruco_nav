#!/usr/bin/env python
import rospy 
from robot_nav.msg import Marker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from math import pi, sin

min_angle = 0.02

increment = min_angle*10
goal = [0,0]

marker_detected = False
marker_search = False
marker_aligned = False
goal_set = False
goal_published = False

goal_stop = rospy.ServiceProxy('ria/odom/goal/stop', SetBool)
goal_reset = rospy.ServiceProxy('ria/odom/goal/reset', Trigger)
odom_reset = rospy.ServiceProxy('ria/odom/reset', Trigger)

pub = rospy.Publisher("/ria/odom/local/goal",Twist, queue_size=1)

def start_search(msg):
    global marker_search, marker_detected, marker_aligned, goal_set, goal_published
    marker_search = msg.data
    if not marker_search:
        marker_detected = False
        marker_aligned = False
        goal_set = False
        goal_published = False
        increment = min_angle*10
        goal = [0,0]
        try:
            marker_detected = True
            goal_stop(True)
            goal_reset()
            odom_reset()
        except rospy.ServiceException as e:
            print("Service call falied: %s"%e)
    response = SetBoolResponse()
    response.success = True
    rospy.loginfo("Searching Set: {}".format(marker_search))
    return response

def detectCallback(msg):
    global increment, goal_set, marker_aligned, marker_detected, marker_search, goal_stop, goal_reset, odom_reset
    if not marker_search:
        return
    increment = min_angle
    marker_aligned = msg.alinged
    if not marker_detected:
        try:
            marker_detected = True
            goal_stop(True)
            goal_reset()
            odom_reset()
        except rospy.ServiceException as e:
            print("Service call falied: %s"%e)
    elif  marker_aligned and not goal_set:
        try:
            marker_detected = True
            goal_stop(True)
            goal_reset()
            odom_reset()
        except rospy.ServiceException as e:
            print("Service call falied: %s"%e)

        goal[0] = ((90 -abs(msg.theta))*_pi)/180
        if msg.theta <0:
            goal[0] = -1*goal[0]
        goal[1] = msg.distance*sin(abs(msg.theta))
        goal_set = True

def posCallback(msg):
    global marker_search, goal_set, increment, pub, goal_published
    s_ang = msg
    if not marker_search or goal_published:
        return
    if not goal_set:
        s_ang.angular.z += increment
        pub.publish(s_ang)
    else:
        goal_published = True
        s_ang.angular.z = goal[0]
        s_ang.linear.x = goal[1]

def listner():
    rospy.init_node('marker_search', anonymous=True)

    rospy.wait_for_service('ria/odom/goal/stop')
    rospy.wait_for_service('ria/odom/goal/reset')
    rospy.wait_for_service('ria/odom/reset')

    rospy.Subscriber('/ria/odom/marker', Marker, detectCallback)
    rospy.Subscriber('/ria/odom/local', Twist, posCallback)
    m_search = rospy.Service('ria/odom/marker/search',SetBool, start_search)
    rospy.spin()

if __name__=='__main__':
    listner()

