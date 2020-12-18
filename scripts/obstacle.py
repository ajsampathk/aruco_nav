#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco 
import numpy as np
import sys
import os
from std_msgs.msg import Bool
class ImageListener:
	
    def __init__(self, depth_topic,color_topic=None,robot_width=1200):
        self.depth_topic = depth_topic
        self.color_topic = color_topic
        self.robot_width = robot_width

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()



        self.bridge = CvBridge()
        self.depth_sub = message_filters.Subscriber(depth_topic, msg_Image)
        self.color_sub = message_filters.Subscriber(color_topic, msg_Image)
        self.sync = message_filters.TimeSynchronizer([self.depth_sub,self.color_sub],1)
        self.sync.registerCallback(self.imagesCallback)
        self.VISUALIZE= True
        self.image_pub = rospy.Publisher('/mask/image',msg_Image,queue_size=1)
        self.obstacle_pub = rospy.Publisher("/ria_nav/obstacle",Bool,queue_size=1)
        rospy.loginfo("Initialized Listener")
        rospy.loginfo("Visualization: {}".format(self.VISUALIZE))

    def detectObstacle(self):
            threshold = 1000
            center_distance = self.cv_depth_image[self.mid[0],self.mid[1]]
            row,col = np.where(self.cv_depth_image<threshold)
            points = np.vstack((row,col)).T

            try:
                right_limit = int(340+(485*((self.robot_width/2.0)/center_distance)))
                left_limit = int(340-(485*((self.robot_width/2.0)/center_distance)))
                if right_limit>=640:
                    right_limit=639
                if left_limit<0:
                    left_limit=0
            except OverflowError as e:
                rospy.logwarn(e)
                right_limit=self.image_width-1
                left_limit = 0

            left = points>=(0,left_limit)
            right = points<=(self.image_height,right_limit)
            total_points = np.logical_and(left,right)
            population = total_points.sum()-len(total_points)
            density = float(population)/float(self.image_height*(right_limit-left_limit))


            if self.VISUALIZE:
                
                #center
                self.self.cv_color_image[mid[1]-20:mid[1]+20,mid[0]]=0
                self.cv_color_image[mid[1],mid[0]-20:mid[0]+20]=0

                #distance coloring
                self.cv_color_image[row,col,0]=128

                #left axis
                self.cv_color_image[:,0:left_limit,1]=150
                self.cv_color_image[:,left_limit]=255

                #right axis
                self.cv_color_image[:,right_limit:,1]=150
                self.cv_color_image[:,right_limit]=255

                #info box
                self.cv_color_image[25:130,5]=255
                self.cv_color_image[25:130,150]=255
                self.cv_color_image[25,5:150]=255
                self.cv_color_image[130,5:150]=255
                
                font=cv2.FONT_HERSHEY_TRIPLEX
                fontScale = 0.32
                thickness = 1

                range_org = (10,45)
                density_org = (10,105)
                fov_org =(10,87)
                r_width_org = (10,65)
                obs_org = (10,125)
                center_org=(mid[0]+15,mid[1]+15)

                white=(255,255,255)
                yellow=(255,255,0)
                
                color=white
                if density>0.039:
                    obs_color=(255,0,0)
                else:
                    obs_color=(0,255,0)
                
                range_str = "Threshold: {:.2f}m".format(threshold/1000.0)
                r_width_str = "R_width: {:.2f}m".format(self.robot_width/1000.0)
                obs_str = "Obstacle: "+str(density>0.039) 
                density_str = "Density: {:.2f}%".format(density*100.0)
                center_str = "{:.2f}m".format(center_distance/1000.0)  
                fov_str = "FOV: {} [{:.2f}%]".format((right_limit-left_limit),100*float(right_limit-left_limit)/float(color_image.width))  
                
                self.cv_color_image = cv2.putText(self.cv_color_image,obs_str,obs_org,font,fontScale,obs_color,thickness,cv2.LINE_AA)
                self.cv_color_image = cv2.putText(self.cv_color_image,range_str,range_org,font,fontScale,yellow,thickness,cv2.LINE_AA)
                self.cv_color_image = cv2.putText(self.cv_color_image,fov_str,fov_org,font,fontScale,color,thickness,cv2.LINE_AA)
                self.cv_color_image = cv2.putText(self.cv_color_image,density_str,density_org,font,fontScale,color,thickness,cv2.LINE_AA)
                self.cv_color_image = cv2.putText(self.cv_color_image,center_str,center_org,font,fontScale,color,thickness,cv2.LINE_AA)
                self.cv_color_image = cv2.putText(self.cv_color_image,r_width_str,r_width_org,font,fontScale,yellow,thickness,cv2.LINE_AA)

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_color_image,"rgb8"))

                except CvBridgeError as e:
                    rospy.logerr(e)

            print("[DENSITY]:{} [Obstacle]:{} |Center:{}".format(density,density>0.039,center_distance))
            if density>0.039:
              self.obstacle_pub.publish(True)
            else:
              self.obstacle_pub.publish(False)

    def detectMarker(self):
        frame = self.cv_color_image
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,rejects = aruco.detectMarkers(gray,self.aruco_dict,parameters=self.aruco_params)
        
        # rospy.loginfo(ids[0])
        
        if corners:
            
            _b = corners[0][0][3][1]-corners[0][0][0][1]
            _a = corners[0][0][2][1]-corners[0][0][1][1]
            _h = int(((corners[0][0][1][0]-corners[0][0][0][0])+(corners[0][0][3][0]-corners[0][0][2][0]))/2.0)

            center_y = int((corners[0][0][3][1]-corners[0][0][0][1]) + corners[0][0][0][1])
            delta_x = (((_b+2.0*_a)/(3.0*(_a+_b)))*_h)
            center_x = int(delta_x+(corners[0][0][0][0]+corners[0][0][3][0])/2.0)
            _distance = float(self.cv_depth_image[center_x,center_y])/1000.0  

            if self.VISUALIZE:
                font=cv2.FONT_HERSHEY_TRIPLEX
                fontScale = 0.32
                thickness = 1
                yellow=(255,255,0)
                aruco.drawDetectedMarkers(frame,corners)
                frame = cv2.putText(frame,str(_distance),(int(center_x),int(center_y)-5),font,fontScale,yellow,thickness,cv2.LINE_AA)
                frame = cv2.putText(frame,str(ids[0]),(int(corners[0][0][3][0]),5+int(corners[0][0][3][1])),font,fontScale,(255,255,255),thickness,cv2.LINE_AA)

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame,"rgb8"))
                    rospy.loginfo("Marker Image Published")
                except CvBridgeError as e:
                    rospy.logerr(e)
            else:
                rospy.loginfo("Markers not found")




    def imagesCallback(self,depth_image,color_image):
        try:
            self.cv_color_image = self.bridge.imgmsg_to_cv2(color_image,color_image.encoding)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image,depth_image.encoding)
            self.mid = (color_image.width/2,color_image.height/2)
            self.image_width = color_image.width
            self.image_height = color_image.height
            #self.detectObstacle()
            self.detectMarker()
            
        except CvBridgeError as e:
            print(e)
            return

def main():
    color_topic = '/camera/color/image_raw'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'
    listener = ImageListener(depth_topic,color_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
