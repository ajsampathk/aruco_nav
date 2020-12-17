import rospy
# from geometry_msgs.msg import Twist
from dynamixel_workbench_msgs.msg import DynamixelStateList
i_right,i_left = 0,0
first = True

def odom(msg):
    global i_right, i_left, first
    wheels = {}
    for i in msg.dynamixel_state:
        wheels[i.name] = i.present_position
    if first:
        i_right,i_left = (wheels["Right_Rear"]+wheels["Right_Front"])/2,(wheels["Left_Rear"]+wheels["Left_Front"])/2 
    else:
        right,left = (wheels["Right_Rear"]+wheels["Right_Front"])/2 - i_right ,(wheels["Left_Rear"]+wheels["Left_Front"])/2 - i_left
    linear = ((abs(right)+ abs(left))/2)*right/abs(right)
    rotational = (abs(right)+ abs(left))/15
    rospy.loginfo("")

def listener():
    rospy.init_node('ria_odom',anonymous=True) 
    rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,odom)
    rospy.spin()

if __name__=='__main__':
    listener()
