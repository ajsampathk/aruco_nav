import rospy
import message_filters
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys
import os

class ImageListener:
	
    def __init__(self, depth_topic,color_topic=None,robot_width=1200):
        self.depth_topic = depth_topic
        self.color_topic = color_topic
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber(depth_topic, msg_Image)
        self.color_sub = message_filters.Subscriber(color_topic, msg_Image)
        self.sync = message_filters.TimeSynchronizer([self.depth_sub,self.color_sub],1)
        self.sync.registerCallback(self.imagesCallback)
        self.image_pub = rospy.Publisher('/mask/image',msg_Image,queue_size=1)

    def imagesCallback(self,depth_image,color_image):
        try:
            cv_color_image = self.bridge.imgmsg_to_cv2(color_image,color_image.encoding)
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,depth_image.encoding)
            threshold = 1000
            mid = (color_image.width/2,color_image.height/2)
           #center axis
            cv_color_image[:,mid[0]+10]=0
            center_distance = cv_depth_image[mid[0],mid[1]]
            try:
                right_limit = int(340+(485*((robot_width/2.0)/center_distance)))
                left_limit = int(340-(485*((robot_width/2.0)/center_distance)))
                if right_limit>640:
                    right_limit=639
                if left_limit<0:
                    left_limit=0
            except OverflowError:
                right_limit=color_image.width-1
                left_limit = 0
            x,y = np.where(cv_depth_image<threshold)
            cv_color_image[x,y,0]=128

            #left axis
            cv_color_image[:,0:left_limit,2]=128
            cv_color_image[:,left_limit]=255

            #right axis
            cv_color_image[:,right_limit:,2]=128
            cv_color_image[:,right_limit]=255

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_color_image,"rgb8"))
                rospy.loginfo("Center:{}M , LL:{} , RL:{}M".format(center_distance,left_limit,right_limit))

            except CvBridgeError as e:
                rospy.logerr(e)
        except CvBridgeError as e:
            print(e)
            return

def main():
    color_topic = '/camera/color/image_rect_color'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'
    listener = ImageListener(depth_topic,color_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
