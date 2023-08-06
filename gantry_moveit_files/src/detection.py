#!/usr/bin/env python3

import rospy, tf, rospkg, random,tf2_ros
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
from sensor_msgs.msg import Image, LaserScan # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import *
import numpy as np
import math, time
import sys
# from GetBoolVal.srv import *
# from ConveyorBeltControl.srv import *

stop = False 
power = 0.5

# def get_bool_val_client(stop):
#     rospy.wait_for_service('get_bool_val')
#     try:
#         get_bool_val = rospy.ServiceProxy('get_bool_val', GetBoolVal)
#         resp1 = get_bool_val(stop)
#         return resp1.res
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# def power_client(stop):
#     rospy.wait_for_service('gazebo_conveyor/ConveyorBeltControl')
#     try:
#         belt = rospy.ServiceProxy('/conveyor/control', ConveyorBeltControl)
#         resp1 = belt(power)
#         return resp1.success
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)


def callback_state(msg):
    global stop
    range_val = msg.ranges[0]
    try:
        scan = rospy.Publisher("/box/scan", Bool, queue_size =10)
    except(Exception):
        print(Exception)
    
    if range_val < 0.7:
        stop = True
        power = 0.0
    else:
        stop = False
        power = 0.5
    # resp = get_bool_val_client(stop)
    # resp = power_client(power)
    # print(stop)
    scan.publish(stop)
    # time.sleep(1)
    # if resp == True:



    print(range_val)

def receive_message():

    global camera_wrt_origin
    rospy.init_node('data_accept', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer) 
    rate = rospy.Rate(20.0)
    # Node is subscribing to the video_frames topic
    # rospy.Subscriber('/camera2/color/image_raw', Image, callback)

    # try:
    # trans = tfBuffer.lookup_transform('world', 'Base', rospy.Time())
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    # print(trans)
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('/cam_on_laser/scan', LaserScan, callback_state)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue



if __name__ == '__main__':
  receive_message()