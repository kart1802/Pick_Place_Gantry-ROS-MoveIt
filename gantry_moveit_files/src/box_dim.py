#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
from sensor_msgs.msg import Image

def detect_coordinate(x, y, w, h, Average_depth,hfov_rad):
  # global Average_depth, x, y, w, h
  # img_width: pixel image width
  img_width = 640
  img_height = 480
  # h_fov_rad: Horizontal Field of View of Camera in Radians
  # focal_length: constant focal length of camera
  focal_length = (img_width/2)/math.tan(hfov_rad/2)

  # for (x, y, w, h) in logo:
  #   cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
    # centre_x_pixel= (2*x+w)/2.0
    # centre_y_pixel= (2*y+h)/2.0
  centre_x_pixel = x
  centre_y_pixel = y
  err_x_m = ((img_width/2)-centre_x_pixel)*(Average_depth)/focal_length
  err_y_m = ((img_height/2)-centre_y_pixel)*(Average_depth)/focal_length
  err_h = (h)*(Average_depth)/focal_length
  err_w = (w)*(Average_depth)/focal_length
  return(err_x_m,err_y_m, err_h, err_w)

def estimate_depth(depth_image):


    depth_image_8bit = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)

    min_contour_area = 10
    max_contour_area = 5000
    min_aspect_ratio = 0.8
    max_aspect_ratio = 1.2
    color = (0,0,0)
    x = 0
    y = 0
    w = 0
    h = 0
    Orientation = 0

    # depth_image_8bit = np.uint8(depth_image / np.max(depth_image) * 255)
    hsv = cv2.cvtColor(depth_image, cv2.COLOR_BGR2HSV) 
    min_green = np.array([0,0,0]) 
    max_green = np.array([102,255,255]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(depth_image,depth_image, mask= mask_g)
    edges = cv2.Canny(image=res_g, threshold1=85, threshold2=255)
    contours, hierarchy= cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    depth = 0
    for contour in contours:
        # Ignore small contours (noise)
        if cv2.contourArea(contour) < min_contour_area and cv2.contourArea(contour) > max_contour_area:
            continue
        # print(cv2.contourArea(contour))

        # Approximate the contour to a rectangle
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the contour is rectangular (4 corners)
        if len(approx) == 4:
            # Calculate the dimensions of the rectangle
            # x, y, w, h = cv2.boundingRect(contour)
            (x,y),(w,h),Orientation = cv2.minAreaRect(contour)
            rect =cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            Orientation_z = Orientation*math.pi/180
            boundRect = cv2.boundingRect(box)
            depth = round(np.average(depth_image_8bit[int(x-1):int(x+1),int(y-1):int(y+1)]),2)/255
            cv2.rectangle(depth_image_8bit, (int(boundRect[0]), int(boundRect[1])),(int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 1)

            # Check if the aspect ratio is within the desired range (near square)
            # Print the estimated dimensions
            rospy.loginfo(f"Detected rectangular object with width: {w} pixels, height: {h} pixels")
    
    Y_Coordinate, Z_Coordinate, box_height, box_width = detect_coordinate(x, y, w, h, depth, 1.047198) 
    print(box_height,box_width,depth)
    cv2.imshow("Depth Image with Boundaries", depth_image_8bit)
    cv2.waitKey(1)


def callback_cam(data):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, "bgr8")
    estimate_depth(depth_image)
    # cv2.imshow("Depth Image with Boundaries", depth_image)
    # cv2.waitKey(1)
    # estimate_depth(depth_image)

def receive_message():
    rospy.init_node("Box_Dimension")
    rospy.Subscriber("/camera1/image_raw",Image, callback_cam)
    rospy.sleep(10)


if __name__ == '__main__':
  receive_message()