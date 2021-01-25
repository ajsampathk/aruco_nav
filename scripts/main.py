#!/usr/bin/env python
import rospy
import sys
import os
from ArucoPerception import ArucoPerception


def main():
    color_topic = '/camera/color/image_raw'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'
    listener = ArucoPerception(depth_topic,color_topic)
    listener.startMarkerDetection()
    listener.startObstacleDetection()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if listener.new_image:
                listener.detectMarker()
                listener.detectObstacle()
                listener.publish_image()
                listener.publish_marker()
            else:
                rospy.logwarn("Waiting for synchronized depth image")
            rate.sleep()
            

    except KeyboardInterrupt:
        listener.stopAll()
        rospy.loginfo("Stopping Node")
        exit()


if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
