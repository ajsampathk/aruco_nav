#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

pub= rospy.Publisher("/dynamixel_workbench/cmd_vel",Twist,queue_size=1)
goal_fb = rospy.Publisher("/ria/odom/goal_fb", Bool, queue_size=1)
goal = [0,0]
rotate, stop, reset = True, False, False
new_goal=False

def goal_stop(msg):
    global stop
    stop = msg.data
    response = SetBoolResponse()
    response.success = True
    response.message = "{} ".format(stop)
    rospy.loginfo("Goal Stopped Set: {}".format(stop))
    return response

def goal_reset(msg):
    global reset, goal
    response = TriggerResponse()
    reset = True
    goal = [0,0]
    rospy.wait_for_service('ria/odom/reset')
    try:
        reset = rospy.ServiceProxy('ria/odom/reset', Trigger)
        resp = reset()
        return resp
    except rospy.ServiceException as e:
        print("Service call falied: %s"%e)
    response.success = True
    rospy.loginfo("Goal Reset")
    return response

def goalCallback(msg):
    global goal,rotate,new_goal
    goal[0] = msg.linear.x
    goal[1] = msg.angular.z
    rotate=True
    new_goal=True
    rospy.loginfo("Goal:{}".format(goal))

def poseCallback(msg):
   global goal,pub,rotate, stop, new_goal
   zero = Twist()
   ang = Twist()
   lin = Twist()
   #rospy.loginfo("{},{},{}".format(stop,rotate,goal))
   if new_goal and rotate:
       th_diff=goal[1]-msg.angular.z
       if abs(th_diff)<0.015:
         pub.publish(zero)
         rotate=False
       else:
         if th_diff <0:
             ang.linear.x = -0.005
         else:
             ang.linear.x = 0.005
         if stop:
             pub.publish(zero)
         else:
             pub.publish(ang)
       #rospy.loginfo("Theta Diff:{}".format(th_diff))
   elif new_goal:
        l_diff = goal[0]-msg.linear.x
        if new_goal and abs(l_diff)<0.025:
            pub.publish(zero)
            goal_fb.publish(True)
            new_goal=False
            rotate=True
        else:
           if l_diff<0:
               lin.angular.z=-0.2
           else:
               lin.angular.z=0.2
           if stop:
               pub.publish(zero)
           else:
               pub.publish(lin)
        rospy.loginfo("Linear diff: {}".format(l_diff))

def listener():
    rospy.init_node('ria_manualControl',anonymous=True)
    g_stop = rospy.Service('/ria/odom/goal/stop',SetBool,goal_stop)
    g_reset = rospy.Service('/ria/odom/goal/reset', Trigger, goal_reset)
    rospy.Subscriber('/ria/odom/local/goal',Twist,goalCallback)
    rospy.Subscriber('/ria/odom/local',Twist, poseCallback)
    rospy.spin()

if __name__=='__main__':
    listener()
