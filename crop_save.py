#!/usr/bin/env python3
import rospy
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def image_callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    color_image = np.array(cv_image, dtype=np.uint8)

    color_image = color_image[:,:,::-1]
    # bgr to rgb
    #print(color_image)
    cv2.imshow('lala', color_image)
    crop_save(color_image)

def crop_save(data):
    k = cv2.waitKey(20)
    if k == ord('s'):
        cv2.imwrite('/home/ethan251/Desktop/lala.png', data)
        print("image saved")
        #cv2.destroyAllWindows()

def showImage():
    rospy.init_node('showImage',anonymous = True)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print('Shutting Down...')
    # cv2.destroyAllWindows()
    rospy.spin()

if __name__ == '__main__':
    showImage()
