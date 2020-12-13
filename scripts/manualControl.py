#!/usr/bin/env/python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
pub= rospy.Publisher("/ria_nav_teleop/cmd_vel",Twist,queue_size=1)

def joyCallback(msg):
    command= Twist()
    axes= msg.axes
    command.angular.z= 0.3*msg.axes[1]
    command.linear.x= 0.005* axes[3]
    pub.publish(command)
    rospy.loginfo("published")



def listener():
    rospy.init_node('ria_manualControl',anonymous=True)
    
    rospy.Subscriber('/joy',Joy,joyCallback)
    rospy.spin()

if __name__=='__main__':
    listener()
