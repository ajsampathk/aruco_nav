#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
goalPublisher= rospy.Publisher("goal_pose",Pose,queue_size=1)
goalPoints=[[0.3,0],[0.6,0],[0.6,-0.3],[0.6,-0.6],[0.3,-0.6],[0,-0.6],[0,-0.3],[0,0],[0.2,0]]

def callback(_msg):
	global goalPoints,goalPublisher
	if _msg.data > len(goalPoints):
		pass
	else:
		msg=Pose()
		curGoal= goalPoints[_msg.data]
		rospy.loginfo("Current Goal:{}".format(curGoal))
		msg.position.x=curGoal[0]
		msg.position.y=curGoal[1]
		goalPublisher.publish(msg)
	
def listener():
	rospy.init_node('ria_goalPublisher',anonymous=True)
	rospy.Subscriber('/ria_nav/goal_feedback',Int8,callback)
	while not rospy.is_shutdown():
		rospy.spin()
if __name__== '__main__':
	listener()
