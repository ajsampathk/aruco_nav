#!/usr/bin/env python
import rospy
from robot_nav.msg import Marker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
goal = [0,0]
marker_search = False
marker_detected = False
marker_alinged = False

pub = rospy.Publisher("/ria/odom/local/goal",Twist, queue_size=1)

def detectCallback(msg):
    global marker_detected, marker_alinged
    if  marker_detected and not marker_alinged:
        marker_alinged = msg.alinged
        return
    marker_detected = True
    rospy.wait_for_service('ria/odom/goal/stop')
    try:
        goal_stop = rospy.ServiceProxy('ria/odom/goal/stop', Trigger)
        resp = goal_stop()
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

def start_search(mgs):
    global marker_search
    marker_search = msg.data
    response = SetBoolResponse()
    response.success = True
    rospy.loginfo("Searching Set: {}".format(marker_search))
    return response

def posCallback(msg):
    global marker_search, marker_detected, marker_alingned
    s_ang = msg
    if marker_search:
        if  marker_detected and not marker_alinged:
            s_ang.angular.z += 0.5
        elif not marker_search:
            s_ang.angular.z += 1
        else:
            s_ang = Twist()
        pub.publish(s_ang)

def listner():
    rospy.Subscribe('/ria/odom/marker',Marker, detectCallback)
    rospy.Subscribe('/ria/odom/local',Twist, posCallback)
    m_search = rospy.Service('ria/odom/marker/search',SetBool, start_search)
    rospy.spin()
    #while not rospy.is_shutdown():
    #    goal_publish()
if __name__=='__main__':
    listner()
