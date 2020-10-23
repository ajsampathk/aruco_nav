#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose,Twist
import tf
from math import atan,sqrt,pi
from tf import transformations
import threading
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8

lost = False
goal_x= None
goal_y= None
cur_x= None
cur_y= None
goal_flag=None
timer=None
pointCounter=0
pub= rospy.Publisher("/dynamixel_workbench/cmd_vel",Twist,queue_size=1)
goalfb=rospy.Publisher("/ria_nav/goal_feedback",Int8,queue_size=1)

def track_check(data):
	global timer
	timer.cancel()
	timer=threading.Timer(5,timeout)
	timer.start()

def timeout():
	global timer,pub,lost
	rospy.logwarn("Timeout")
	pub.publish(Twist())
	lost=True



def linearCorrection():
	global goal_x,goal_y,cur_x,cur_y,goal_flag,pub,goalfb,pointCounter
	lin_goal= sqrt((goal_x)**2+(goal_y)**2)
	lin_cur= sqrt((cur_x)**2+(cur_y)**2)
	lin_diff= lin_goal-lin_cur
	rospy.loginfo("Linear Difference: {}".format(lin_diff))
	command=Twist()
	if abs(lin_diff) < 0.05:
		rospy.loginfo("Linear Done")
		pointCounter+=1
		command.angular.z=0
		pub.publish(command)
		goalfb.publish(pointCounter)
		goal_flag=2
	else:
		command.angular.z=0.3
		pub.publish(command)

def calcGoal():
	global goal_x,goal_y,cur_x,cur_y
	x_diff= goal_x-cur_x
	y_diff= goal_y-cur_y
	if x_diff>0:
		return atan(y_diff/x_diff)
	elif x_diff<0:
		if y_diff<0:
			return atan(y_diff/x_diff)-pi
		elif y_diff>0:
			return atan(y_diff/x_diff)+pi
		else:
			return pi
	else:
		if y_diff>0:
			return pi/2
		elif y_diff<0:
			return -pi/2
		else:
			return None

def angularCorrection(y):
	global goal_x,goal_y,cur_x,cur_y,goal_flag,pub

	goal_angle= calcGoal()
	rospy.loginfo("!!Goal:{} Yaw: {} !!".format(goal_angle,y))
#	rospy.loginfo("Difference: {}".format(goal_angle))
#	rospy.loginfo("R:{},P:{},Y:{}".format(r,p,y))
	ang_diff= goal_angle-y
	rospy.loginfo("Current Diff= {}".format(ang_diff))
	if abs(ang_diff)>0.05:
		if ang_diff <0:
			rospy.loginfo("Clockwise")
			command= Twist()
			command.linear.x=0.005
			pub.publish(command)
		else:
			command=Twist()
			command.linear.x=-0.005
			pub.publish(command)
			rospy.loginfo("CCW")
	else:
		rospy.loginfo("Angular Done")
		command=Twist()
		command.linear.x=0
		pub.publish(command)
		goal_flag=1

def goalCallback(data):
	global goal_x,goal_y,goal_flag
	goal_x=data.position.x
	goal_y=data.position.y
	rospy.loginfo("Received {},{}".format(goal_x,goal_y))
	goal_flag=0

def positionCallback(lin,rot):
	global cur_x,cur_y
	cur_x= lin[0]
	cur_y= lin[1]
	#quat=(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)
	r,p,y= transformations.euler_from_quaternion(rot)
	if goal_flag==0:
		angularCorrection(y)
	elif goal_flag==1:
		linearCorrection()
	#rospy.loginfo("Current position {},{}".format(cur_x,cur_y))
def listener():
	global lost,timer
	rospy.init_node('ria_nav',anonymous=True)
	rospy.Subscriber("goal_pose",Pose,goalCallback)
	rospy.Subscriber("/rtabmap/odom_last_frame",PointCloud2,track_check)
	listener=tf.TransformListener()
#	rospy.Subscriber("/orb_slam2_rgbd/pose",PoseStamped,positionCallback)
	timer = threading.Timer(5,timeout)
	timer.start()
	rate = rospy.Rate(10)
	goalfb.publish(0)
	while not rospy.is_shutdown():
		try:
			(lin,rot)=listener.lookupTransform('/odom','/camera_link',rospy.Time(0))
			#positionCallback(lin,rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		if not lost:
			positionCallback(lin,rot)
		else:
			rospy.logwarn("Resetting Odom")
			res=rospy.ServiceProxy("/rtabmap/reset_odom",Empty)
			pub.publish(Twist())
		rate.sleep()
	#	rospy.loginfo("Waiting for current pose")
	#	pub.publish(Twist())
	#	rate.sleep()

if __name__== '__main__':
	rospy.wait_for_service("/rtabmap/reset_odom",timeout=5.0)

	listener()
