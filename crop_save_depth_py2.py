#!/usr/bin/env python
import rospy
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

import message_filters
from message_filters import TimeSynchronizer, Subscriber

from cv_bridge import CvBridge
import cv2
import numpy as np

import time

# def color_callback(data):
#     print('color_time_stamp: ', data.header.stamp)
#     bridge = CvBridge()
#     color_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
#     color_image = np.array(color_image, dtype=np.uint8)
#     color_image = color_image[:,:,::-1]  # bgr to rgb
#     cv2.imshow('color_img', color_image)
#     crop_save_color(color_image)

# def depth_callback(data):
#     print('depth_time_stamp: ', data.header.stamp)
#     bridge = CvBridge()
#     depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
#     depth_image = np.array(depth_image, dtype=np.float32)
#     cv2.imshow('depth_image', depth_image)
#     crop_save_depth(depth_image)


def callback(color_img, depth_img):
    
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(color_img, desired_encoding='passthrough')
    # print 'color_time_stamp: ', color_img.header.stamp
    color_image = np.array(color_image, dtype=np.uint8)
    color_image = color_image[:,:,::-1]  # bgr to rgb
    cv2.imshow('color_img', color_image)
    
    depth_image = bridge.imgmsg_to_cv2(depth_img, desired_encoding='passthrough')
    # print'depth_time_stamp: ', depth_img.header.stamp
    depth_image = np.array(depth_image, dtype=np.float32)
    cv2.imshow('depth_image', depth_image)

    crop_save(color_image, depth_image)


# def crop_save_color(data):
#     k = cv2.waitKey(20)
#     if k == ord('s'):
#         cv2.imwrite('/home/ethan251/Desktop/Color.png', data)
#         print("Color image saved")
#         cv2.destroyAllWindows()

# def crop_save_depth(data):
#     k = cv2.waitKey(20)
#     if k == ord('s'):
#         cv2.imwrite('/home/ethan251/Desktop/Depth.exr', data)
#         print("Depth image saved")
#         cv2.destroyAllWindows()


def crop_save(color_img, depth_img):
    k = cv2.waitKey(20)
    if k == ord('s'):
        cv2.imwrite('/home/ethan251/Desktop/color'+ str(int(time.time())) +'.png', color_img)
        cv2.imwrite('/home/ethan251/Desktop/depth'+ str(int(time.time())) +'.exr', depth_img)
        print("Both color and depth images at time successfully saved.")
        cv2.destroyAllWindows()

def saveImage():

    rospy.init_node('showImage',anonymous = True)
    # rospy.Subscriber('/camera/color/image_raw', Image, color_callback)
    # rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)

    color_img = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_img = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

    ts = message_filters.TimeSynchronizer([color_img, depth_img], 10)
    ts.registerCallback(callback)
    # Synchronize time stamps of color and depth images

    rospy.spin()

if __name__ == '__main__':
    saveImage()
