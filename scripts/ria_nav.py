#!/usr/bin/env/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

obstacle=False
cmd= Twist()
pub= rospy.Publisher("/dynamixel_workbench/cmd_vel",Twist,queue_size=1)

def obstacleCallback(msg):
	global obstacle
	obstacle= msg.data
	rospy.loginfo(obstacle)

def joyCallback(msg):
	global cmd
	rospy.loginfo("Joy Working")
	cmd=msg

def listener():
	global obstacle,cmd
	zero_cmd=Twist()
	rospy.init_node('ria_base_controller',anonymous=True)
	rospy.Subscriber('ria_nav/obstacle',Bool,obstacleCallback)  #from obstacle detection node
	rospy.Subscriber('ria_nav_teleop/cmd_vel',Twist,joyCallback) #from joystick
	#rospy.Subscriber('ria_nav/cmd_vel',Twist,navCallback)  #from navigation node
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		if not obstacle:
			pub.publish(cmd)
		else:
			cmd.angular.z=0
			pub.publish(cmd)
		rate.sleep()

if __name__=='__main__':
	listener()









