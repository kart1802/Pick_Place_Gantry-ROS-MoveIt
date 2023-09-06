#! /usr/bin/env python3
import rospy, tf, rospkg, random,tf2_ros, cv2
import numpy as np
from cv_bridge import CvBridge
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from std_msgs.msg import *
from sensor_msgs.msg import Image, LaserScan 
# from DimDataArray.msg import DimDataArray 

scan_trigger = False
depth_val = [0.0,0.0]
temp = 0.0
box_dim = [0.0,0.0,0.0,0.0]
transf = []

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
    global box_dim
    depth_image_8bit = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    # print("Checking image")
    dim_pub = rospy.Publisher("/box/dim",Float64MultiArray,queue_size=10)
    len_pub = rospy.Publisher("/box/len",Float64,queue_size=10)
    dimension = Float64MultiArray()
    # dimension_array = DimDataArray()
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
    box_height = 0

    # depth_image_8bit = np.uint8(depth_image / np.max(depth_image) * 255)
    hsv = cv2.cvtColor(depth_image, cv2.COLOR_BGR2HSV) 
    min_green = np.array([0,100,100]) 
    max_green = np.array([10,255,255]) 

    mask_g = cv2.inRange(hsv, min_green, max_green) 

    res_g = cv2.bitwise_and(depth_image,depth_image, mask= mask_g)
    # cv2.imshow("RESG",res_g)
    edges = cv2.Canny(image=res_g, threshold1=0, threshold2=255)
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
            # print(depth_image_8bit[int(x),int(y)])
            depth = round(np.average(depth_image_8bit[int(x-1):int(x+1),int(y-1):int(y+1)]),1)/255
            box_height = 0.4 - depth
            cv2.drawContours(depth_image_8bit,[box], 0, (0,0,255),1)
            cv2.circle(depth_image_8bit,(int(x),int(y)), 2, (0,0,255), -1)
            # cv2.drawContours(depth_image_8bit,[int(x-1),int(x+1),int(y-1),int(y+1)], 0, (0,0,255),1)
            # cv2.rectangle(depth_image_8bit, (int(boundRect[0]), int(boundRect[1])),(int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 1)

            # Check if the aspect ratio is within the desired range (near square)
            # Print the estimated dimensions
            # rospy.loginfo(f"Detected rectangular object with width: {w} pixels, height: {h} pixels")
            break
    
    Y_Coordinate, Z_Coordinate, box_length, box_width = detect_coordinate(x, y, w, h, depth, 1.047198) 
    # print(box_length,box_width,depth, Orientation)
    dim = [box_length,box_width,box_height,Orientation]
    dimension.data = dim
    print(dimension)
    if 270 < y < 280:
      box_dim = dim
      dim_pub.publish(dimension)
      len_pub.publish(w)
      print(dimension)
    cv2.imshow("Depth Image with Boundaries", depth_image_8bit)
    cv2.waitKey(1)


def callback_cam(data):
    global scan_trigger
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # print(scan_trigger)
    if scan_trigger == True :
      # print("yay")
      estimate_depth(depth_image)
    # cv2.imshow("Depth Image with Boundaries", depth_image)
    # cv2.waitKey(1)
    # estimate_depth(depth_image)

def callback_scan(data):
  global scan_trigger
  if data.data == True and scan_trigger == False:
    scan_trigger = True
  elif data.data == False and scan_trigger == True:
    scan_trigger = False

def depth_state(msg):
    global depth_val,temp,transf
    range_val = msg.ranges[0]

    try:
        scan = rospy.Publisher("/depth/scan", Float64, queue_size =10)
        loc = rospy.Publisher('/box/loc',Float64MultiArray,queue_size=1)
        num = Float64()
        box_loc = Float64MultiArray()
    except(Exception):
        print(Exception)
    
    depth_val[0] = temp
    depth_val[1] = range_val
    if range_val < 0.39:
      d = abs(depth_val[1] - depth_val[0])
      num.data = d
      scan.publish(num)
      x = box_dim[1]*math.cos(box_dim[3]) - transf[0] 
      y = 0.375 + transf[1] 
      z = 0.589 + d - transf[2] + 0.02
      w = 0
      box_loc.data = [x,y,z,w]
      loc.publish(box_loc)
    else:
      temp = range_val
    

def receive_message():
    global transf
    rospy.init_node("Box_Dimension")
    rospy.Subscriber("/camera1/image_raw",Image, callback_cam)
    rospy.Subscriber("/box/scan", Bool, callback_scan)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer) 
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world','Cup',rospy.Time())
            transf = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
            rospy.Subscriber('/laser/scan', LaserScan, depth_state)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue



if __name__ == '__main__':
  receive_message()