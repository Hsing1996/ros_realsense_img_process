#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sys
ros_packages = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_packages in sys.path:
    sys.path.remove(ros_packages)
import numpy as np
import cv2
from copy import deepcopy
import math
import time
from maskrcnn_benchmark.config import cfg
from predictor_bobbin import BobbinsDemo

def setCamera(px=644.236, py=365.787, fx=897.311, fy=897.311):
    #kinect: 325.26110, 242.04899, 572.41140, 573.57043
    camera = {'px':px, 'py':py, 'fx':fx, 'fy':fy}
    return camera

def getXYZ(depth, camera, row, col):
    #camera = setCamera()
    depth = np.array(depth, dtype='float')
    row = int(np.round(row))
    col = int(np.round(col))
    x = 0
    y = 0
    z = 0
    if row <= depth.shape[0] and col <= depth.shape[1]:
        if depth[row][col] != 0:
            z = depth[row][col] / 1000
            x = z * (col + 1 - camera['px']) / camera['fx']
            y = z * (row + 1 - camera['py']) / camera['fy']
        else:
            print("depth is zero")
    else:
        print("invalid row and col")
    return x, y, z

def getNormal(depth, row, col, n):
    l = int(np.floor(0.5*n))
    h = int(np.ceil(0.5*n))
    row = int(np.round(row))
    col = int(np.round(col))
    camera = setCamera()
    roi = []
    for i in range(row - l, row + h):
        for j in range(col - l, col + h):
            x, y, z = getXYZ(depth, camera, i, j)
            roi.append(np.array((x, y, z)))
    roi = np.array(roi)
    normals = cv2.ppf_match_3d.computeNormalsPC3d(roi, 10, False, (0,0,0)) # important param
    normals = normals[1].reshape(n, n, 6)
    normal = normals[l][l]
    return roi, normals, normal

def getWidth(mask, kp):
    head = kp[0, 0:2]
    body = kp[1, 0:2]
    theta = math.atan2(head[1]-body[1], head[0]-body[0])
    am = cv2.getRotationMatrix2D((0.5*(mask.shape[1]-1), 0.5*(mask.shape[0]-1)), 180*theta/math.pi, 1)
    rotate = cv2.warpAffine(mask, am, (mask.shape[1], mask.shape[0]))
    tmp = np.hstack((head, 1))
    tmp2 = np.vstack((am, np.array((0,0,1))))
    head_new = am.dot(tmp)
    line = rotate[:, int(np.round(head_new[0]))]
    index = np.argwhere(line==1)
    if (len(index) == 0):
        print("failed to compute width")
        return np.zeros(2), np.zeros(2), 0
    else:
        up = np.linalg.solve(tmp2, np.array((head_new[0], index[0][0], 1)))
        down = np.linalg.solve(tmp2, np.array((head_new[0], index[-1][0], 1)))
        width = len(np.argwhere(line==1))
        return up[0:2], down[0:2], width

def getAvgHeight(depth, segm):
    roi = cv2.bitwise_and(depth, depth, mask=segm)
    index = np.argwhere(segm == 1)
    height = np.sum(roi) / len(index)
    return height

def getDistance(image, kp):
    head = kp[0, 0:2]
    body = kp[1, 0:2]
    mid = 0.5 * (head + body)
    distance = math.pow((0.5*(image.shape[1]-1) - mid[0]), 2) + math.pow((0.5*(image.shape[0]-1) - mid[1]), 2)
    return distance

def detectGraspPoint(config, color, depth):
    start = time.time()
    #config_path = "/home/xia/maskrcnn-benchmark/configs/e2e_keypoint_rcnn_R_101_FPN_1x_predictor.yaml"
    # update the config options with the config file
    cfg.merge_from_file(config)
    # manual override some options
    cfg.merge_from_list(["MODEL.DEVICE", "cuda"])
    bobbin_demo = BobbinsDemo(cfg,
                              min_image_size=720,
                              confidence_threshold=0.95,
                              )
    # compute predictions
    result, masks, keypoints = bobbin_demo.run_on_opencv_image(color)
    dl = time.time()
    print("deep learning inference uses %.4f seconds." %(dl - start))
    kps = []
    kp_thres = 2
    grasp = []
    #grasp_name = {"0":"x", "1":"y", "2":"z", "3":"nx", "4":"ny", "5":"nz", 
    #              "6":"up_x", "7":"up_y", "8":"down_x", "9":"down_y", "10":"w"}
    heights = []
    for i in range(len(keypoints)):
        if keypoints[i][0][2] > kp_thres and keypoints[i][1][2] > kp_thres:
            kps.append(keypoints[i][:, 0:2])
            _, __, normal = getNormal(depth, keypoints[i][0][1], keypoints[i][0][0], 7)
            up, down, width = getWidth(masks[i], keypoints[i])
            grasp.append(np.hstack((normal, up, down, width)))
            height = getAvgHeight(depth, masks[i])
            heights.append(np.array((i, height)))
            # draw all the grasp points
            cv2.circle(result, (int(np.round(up[0])), int(np.round(up[1]))), 3, (255, 255, 255), -1)
            cv2.circle(result, (int(np.round(down[0])), int(np.round(down[1]))), 3, (255, 255, 255), -1)
    
    heights.sort(key = lambda x:x[1])
    order = [x for x in heights if x[1] < heights[0][1]+20] # important param
    distance = []
    if len(order) >1:
        for i in range(len(order)):
            distance.append(getDistance(color, keypoints[int(order[i][0])]))
        order = np.hstack((np.array(order), np.array(distance).reshape(-1,1))).tolist()
        order.sort(key = lambda x:x[2])
    order.extend(heights[len(order):len(heights)])
    
    end = time.time()
    print("detecting grasp points uses %.4f seconds." %(end - start))
    print("the best grasp point is: ", grasp[int(order[0][0])][0:3])
    return grasp, order, result

def Colorize(depth):
    img = np.float32(depth)
    tmp = deepcopy(img).reshape(-1,)
    index = np.argwhere(tmp>0).flatten()
    tmp[index] = (tmp[index] - np.min(tmp[index])) / (np.max(tmp[index]) - np.min(tmp[index]))
    depth_norm = tmp.reshape(depth.shape[0], depth.shape[1])
    depth_norm = np.uint8(255 * depth_norm)
    colorized = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
    colorized = cv2.cvtColor(colorized, cv2.COLOR_BGR2RGB)
    return colorized

def image_callback(ros_image):
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the color image using the default passthrough encoding
        image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        color = np.array(image, dtype=np.uint8)
        color = color[0:int(0.5*color.shape[0]),:,:]
        depth = np.array(image, dtype=np.float32)
        depth = depth[int(0.5*depth.shape[0]):depth.shape[0],:,0]
        colorized = Colorize(depth)

        display = np.vstack((color, colorized))

        cv2.namedWindow("aligned image", cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("aligned image", int(0.5*display.shape[1]), int(0.5*display.shape[0]))
        cv2.imshow("aligned image", display)
        key = cv2.waitKey(20)
        if key == 32:
            print("detecting grasp points, please wait...")
            config = "/home/xia/maskrcnn-benchmark/configs/e2e_keypoint_rcnn_R_101_FPN_1x_predictor.yaml"
            grasp, order, result = detectGraspPoint(config, color, depth)
            cv2.imwrite("/home/xia/result.png", result)
            #cv2.imwrite("/home/xia/color.png", color)
            #cv2.imwrite("/home/xia/depth.exr", depth)
            #print("images saved.")

        #center_idx = np.array(color.shape) / 2
        #print('color center pixel:', color[int(center_idx[0]), int(center_idx[1])])
        #print('depth center pixel:', depth[int(center_idx[0]), int(center_idx[1])])

    except CvBridgeError as e:
        print(e)

def image_process():
	rospy.init_node('image_process', anonymous=True)
	rospy.Subscriber("/camera/image_concat", Image, callback=image_callback, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
    image_process()
    
