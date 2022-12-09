import rospy
import cv_bridge 
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
import message_filters

def callback(data_left : Image,data_right:Image):
    br = cv_bridge.CvBridge()
    img_left = br.imgmsg_to_cv2(data_left)
    img_right = br.imgmsg_to_cv2(data_right)
    stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(img_left,img_right)
    output = br.cv2_to_imgmsg(disparity)
    pub.publish(output)



if __name__ == '__main__':
    rospy.init_node("depth_image")
    sub_image_left = message_filters.Subscriber("/camera/left/image_raw",Image,queue_size=10)
    sub_image_right = message_filters.Subscriber("/camera/right/image_raw",Image,queue_size=10)
    ts = message_filters.TimeSynchronizer([sub_image_left,sub_image_left], 10)
    ts.registerCallback(callback)
    pub = rospy.Publisher("/camera/disparity",Image,queue_size=10)
    rospy.spin()