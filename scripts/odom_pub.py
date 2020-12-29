#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

pub= rospy.Publisher("/dynamixel_workbench/cmd_vel",Twist,queue_size=1)
goal = [0,0]
rotate = True

def goalCallback(msg):
    global goal,rotate
    goal[0] = msg.linear.x
    goal[1] = msg.angular.z
    rotate=True
    rospy.loginfo("Goal:{}".format(goal))

def poseCallback(msg):
   global goal,pub,rotate
   zero = Twist()
   ang = Twist()
   lin = Twist()
   if rotate:
       th_diff=goal[1]-msg.angular.z
       if abs(th_diff)<0.025:
         pub.publish(zero)
         rotate=False
       else:
         if th_diff <0:
             ang.linear.x = -0.005
         else:
             ang.linear.x = 0.005
         pub.publish(ang)
       #rospy.loginfo("Theta Diff:{}".format(th_diff))
   else:
        l_diff = goal[0]-msg.linear.x
        if abs(l_diff)<0.05:
            pub.publish(zero)
            rotate=True
        else:
           if l_diff<0:
               lin.angular.z=-0.3
           else:
               lin.angular.z=0.3
           pub.publish(lin)
        #rospy.loginfo("Linear diff: {}".format(l_diff))
def listener():
    rospy.init_node('ria_manualControl',anonymous=True)
    
    rospy.Subscriber('/ria/odom/local/goal',Twist,goalCallback)
    rospy.Subscriber('/ria/odom/local',Twist, poseCallback)
    rospy.spin()

if __name__=='__main__':
    listener()

