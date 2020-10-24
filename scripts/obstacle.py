#!/usr/bin/env python
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
        self.robot_width = robot_width
        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber(depth_topic, msg_Image)
        self.color_sub = message_filters.Subscriber(color_topic, msg_Image)
        self.sync = message_filters.TimeSynchronizer([self.depth_sub,self.color_sub],1)
        self.sync.registerCallback(self.imagesCallback)
        self.VISUALIZE= True
        self.image_pub = rospy.Publisher('/mask/image',msg_Image,queue_size=1)
       

    def imagesCallback(self,depth_image,color_image):
        try:
            cv_color_image = self.bridge.imgmsg_to_cv2(color_image,color_image.encoding)
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,depth_image.encoding)
            threshold = 1000
            mid = (color_image.width/2,color_image.height/2)

            center_distance = cv_depth_image[mid[0],mid[1]]
            row,col = np.where(cv_depth_image<threshold)
            points = np.vstack((row,col)).T

            try:
                right_limit = int(340+(485*((self.robot_width/2.0)/center_distance)))
                left_limit = int(340-(485*((self.robot_width/2.0)/center_distance)))
                if right_limit>=640:
                    right_limit=639
                if left_limit<0:
                    left_limit=0
            except OverflowError:
                right_limit=color_image.width-1
                left_limit = 0

            left = points>=(0,left_limit)
            right = points<=(color_image.height,right_limit)
            total_points = np.logical_and(left,right)
            population = total_points.sum()-len(total_points)
            density = float(population)/float(color_image.height*(right_limit-left_limit))


            if self.VISUALIZE:
                
                #center
                cv_color_image[mid[1]-20:mid[1]+20,mid[0]]=0
                cv_color_image[mid[1],mid[0]-20:mid[0]+20]=0

                #distance coloring
                cv_color_image[row,col,0]=128

                #left axis
                cv_color_image[:,0:left_limit,1]=150
                cv_color_image[:,left_limit]=255

                #right axis
                cv_color_image[:,right_limit:,1]=150
                cv_color_image[:,right_limit]=255

                #info box
                cv_color_image[25:100,5]=255
                cv_color_image[25:100,150]=255
                cv_color_image[25,5:150]=255
                cv_color_image[100,5:150]=255
                
                font=cv2.FONT_HERSHEY_TRIPLEX
                fontScale = 0.35
                thickness = 1

#                title_org = (10,40)
                density_org = (10,67)
                fov_org =(10,45)
                obs_org = (10,90)
                center_org=(mid[0]+15,mid[1]+15)
                color=(255,255,255)
                center_color = (255,255,255)
                if density>0.039:
                    obs_color=(255,0,0)
                else:
                    obs_color=color
                
#                title_str = "RIA Obstacle Detection"
                obs_str = "Obstacle: "+str(density>0.039) 
                density_str = "Density: {:.2f}%".format(density*100.0)
                center_str = "{:.2f}m".format(center_distance/1000.0)  
                fov_str = "FOV: "+str(right_limit-left_limit)+"[{:.2f}%]".format(100*float(right_limit-left_limit)/float(color_image.width))  
                
                cv_color_image = cv2.putText(cv_color_image,obs_str,obs_org,font,fontScale,obs_color,thickness,cv2.LINE_AA)
#                cv_color_image = cv2.putText(cv_color_image,title_str,title_org,font,fontScale,color,thickness,cv2.LINE_AA)
                cv_color_image = cv2.putText(cv_color_image,fov_str,fov_org,font,fontScale,color,thickness,cv2.LINE_AA)
                cv_color_image = cv2.putText(cv_color_image,density_str,density_org,font,fontScale,color,thickness,cv2.LINE_AA)
                cv_color_image = cv2.putText(cv_color_image,center_str,center_org,font,fontScale,center_color,thickness,cv2.LINE_AA)
            

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_color_image,"rgb8"))

                except CvBridgeError as e:
                    rospy.logerr(e)

            rospy.loginfo("[DENSITY]:{} [Obstacle]:{} |Center:{}".format(density,density>0.039,center_distance))
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
