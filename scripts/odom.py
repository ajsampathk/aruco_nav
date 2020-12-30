#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import TriggerResponse, Trigger
from dynamixel_workbench_msgs.msg import DynamixelStateList
from math import pi 
_pi = pi*0.1

i_values = {}
right_l, left_l, right_r, left_r  = 0,0,0,0
linear,rotational = 0,0
first = True

local_odom_pub = rospy.Publisher('/ria/odom/local', Twist, queue_size=1)

def reset_initial(msg):
    global first
    response = TriggerResponse()
    first = True
    response.success = True
    rospy.loginfo("Initial value reseted")
    return response

def odom(msg):
    global i_values, right_l, left_l,right_r, left_r, first, linear, rotational
    wheels = {}
    for i in msg.dynamixel_state:
        wheels[i.name] = i.present_position
    if first:
        i_values = wheels
        first = False 
    else:
        right_l = _pi*((((wheels["Right_Rear"] - i_values["Right_Rear"]) + (-1*((wheels["Right_Front"] - i_values["Right_Front"]))))/2.0)/4096)
        left_l = _pi*((((wheels["Left_Rear"] - i_values["Left_Rear"]) + (-1*(wheels["Left_Front"] - i_values["Left_Front"])))/2.0)/4096)
    linear = ((right_l + left_l)/2)
    right_r = (((wheels["Right_Rear"] - i_values["Right_Rear"]) + (wheels["Right_Front"] - i_values["Right_Front"]))/2.0)/4095 
    left_r = (((wheels["Left_Rear"] - i_values["Left_Rear"]) + (wheels["Left_Front"] - i_values["Left_Front"]))/2.0)/4095
    rotational = (((right_r+left_r)/2)*_pi)/0.1
    rospy.loginfo("Linear: "+str(linear)+" Rotatinal: "+str(rotational))
   # rospy.loginfo("R: "+str(right)+" L: "+str(left)) #rotational)
   # rospy.loginfo(wheels)

def listener():
    global linear, rotational
    rospy.init_node('ria_odom',anonymous=True) 
    rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,odom)
    i_reset = rospy.Service('/ria/odom/reset', Trigger, reset_initial)
    rate= rospy.Rate(50)
    while not rospy.is_shutdown():
        pose = Twist()
        pose.linear.x = linear
        pose.angular.z = rotational
        local_odom_pub.publish(pose)
        rate.sleep()

if __name__=='__main__':
    listener()
