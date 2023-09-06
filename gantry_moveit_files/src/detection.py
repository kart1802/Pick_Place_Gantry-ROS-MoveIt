#!/usr/bin/env python3

import rospy, tf, rospkg, random,tf2_ros
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from geometry_msgs.msg import Quaternion, Pose, Point
from sensor_msgs.msg import Image, LaserScan # Image is the message type
from tf2_msgs.msg import TFMessage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import *
import numpy as np
import math, time
import sys
from gantry_moveit_files.srv import *
# from ConveyorBeltControl.srv import *

stop = False 
power = 0.5
i=0
flag = 0

# class robot_gripper():

#     # Constructor
#     def __init__(self):
#         self._attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
#         self._attach_srv_a.wait_for_service()

#         self._attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
#         self._attach_srv_d.wait_for_service()
#         self.model_state_msg = ModelStates()
#         self.box_model_name = 'cube'
#         self.drone_model_name = 'Cup'
#         self.box_index = 0
#         self.drone_index = 0
#         self.pickable_flag = 'False'

#         self.box_coordinates = [0.0, 0.0, 0.0]
#         self.drone_coordinates = [0.0, 0.0, 0.0]
#         rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, self.model_state_callback)
#         self.check_pub = rospy.Publisher('/edrone/gripper_check', String, queue_size=1)
#         self.gripper_service = rospy.Service('/edrone/activate_gripper', Gripper, self.callback_service_on_request)

#     # Destructor
#     def __del__(self):
#         rospy.loginfo('\033[94m' + " >>> Gripper Del." + '\033[0m')

#     def model_state_callback(self, msg):
#         self.model_state_msg.name = msg.name
#         self.model_state_msg.pose = msg.pose
#         self.model_state_msg.twist = msg.twist

#     def callback_service_on_request(self, req):
#         rospy.loginfo('\033[94m' + " >>> Gripper Activate: {}".format(req.activate_gripper) + '\033[0m')
#         rospy.loginfo('\033[94m' + " >>> Gripper Flag Pickable: {}".format(self.pickable_flag) + '\033[0m')

#         if((req.activate_gripper == True) and (self.pickable_flag == 'True') ):
#             self.activate_gripper()
#             return GripperResponse(True)
#         else:
#             self.deactivate_gripper()
#             return GripperResponse(False)

#     def activate_gripper(self):
#         rospy.loginfo("Attach request received")
#         req = AttachRequest()
#         req.model_name_1 = 'robot'
#         req.link_name_1 = 'Cup'
#         req.model_name_2 = 'cube'
#         req.link_name_2 = 'base_link'
#         self._attach_srv_a.call(req)

#     def deactivate_gripper(self):
#         rospy.loginfo("Detach request received")
#         req = AttachRequest()
#         req.model_name_1 = 'robot'
#         req.link_name_1 = 'Cup'
#         req.model_name_2 = 'cube'
#         req.link_name_2 = 'base_link'
#         self._attach_srv_d.call(req)

#     def check(self):
#         try:
#             self.box_index = self.model_state_msg.name.index(self.box_model_name)
#             self.box_coordinates[0] = self.model_state_msg.pose[self.box_index].position.x
#             self.box_coordinates[1] = self.model_state_msg.pose[self.box_index].position.y
#             self.box_coordinates[2] = self.model_state_msg.pose[self.box_index].position.z
#         except Exception as err:
#             self.box_index = -1
#         try:
#             self.drone_index = self.model_state_msg.name.index(self.drone_model_name)
#             self.drone_coordinates[0] = self.model_state_msg.pose[self.drone_index].position.x
#             self.drone_coordinates[1] = self.model_state_msg.pose[self.drone_index].position.y
#             self.drone_coordinates[2] = self.model_state_msg.pose[self.drone_index].position.z
#         except Exception as err:
#             self.drone_index = -1

#         if (self.box_index != -1 and self.drone_index !=-1 ):
#             if(abs(self.drone_coordinates[0] - self.box_coordinates[0]) < 0.1 and abs(self.drone_coordinates[1] - self.box_coordinates[1]) < 0.1 and (self.box_coordinates[2]-self.drone_coordinates[2])>0.105 and (self.box_coordinates[2]-self.drone_coordinates[2])>0):
#                 self.pickable_flag = 'True'
#             else:
#                 self.pickable_flag = 'False'
#         else:
#             self.pickable_flag = 'False'

#         self.check_pub.publish(self.pickable_flag)


def get_bool_val_client(stop):
    rospy.wait_for_service('get_bool_val')
    try:
        get_bool_val = rospy.ServiceProxy('get_bool_val', GetBoolVal)

        resp1 = get_bool_val(stop)
        print("Success!!!!!! \n")
        return resp1.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# def power_client(stop):
#     rospy.wait_for_service('gazebo_conveyor/ConveyorBeltControl')
#     try:
#         belt = rospy.ServiceProxy('/conveyor/control', ConveyorBeltControl)
#         resp1 = belt(power)
#         return resp1.success
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)


def callback_state(msg):
    global stop,i
    range_val = msg.ranges[0]
    try:
        scan = rospy.Publisher("/box/scan", Bool, queue_size =10)
    except(Exception):
        print(Exception)
    
    if range_val < 0.7:
        if i == 0:
            # print("helo")
            stop = True
            power = 0.0
            resp = get_bool_val_client(stop)
            if resp == True :
                i = 1
        # print(resp)
    else:
        i = 0
        stop = False
        power = 0.5
    # resp = get_bool_val_client(stop)
    # resp = power_client(power)
    # print(stop)
    scan.publish(stop)
    # time.sleep(1)
    # if resp == True:

# def tf_callback_state(msg):
#     global stop,i
#     x_translation = msg.transforms[0].transform.translation.x
    # print(x_translation)
    # try:
    #     scan = rospy.Publisher("/box/scan", Bool, queue_size =10)
    # except(Exception):
    #     print(Exception)
    
    # if range_val < 0.7:
    #     if i == 0:
    #         # print("helo")
    #         stop = True
    #         power = 0.0
    #         resp = get_bool_val_client(stop)
    #         if resp == True :
    #             i = 1
    #     # print(resp)
    # else:
    #     i = 0
    #     stop = False
    #     power = 0.5
    # # resp = get_bool_val_client(stop)
    # # resp = power_client(power)
    # # print(stop)
    # scan.publish(stop)
    # # time.sleep(1)
    # # if resp == True:

    # print(range_val)

def grasp_callback_state(msg):
    if msg.data == True:
        activate_gripper()
        flag = 1
        # return GripperResponse(True)
    elif msg.data == False and flag == 1:
        deactivate_gripper()
        flag = 0


def activate_gripper():

    rospy.loginfo("Attach request received")
    attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv_a.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = 'robot'
    req.link_name_1 = 'Cup'
    req.model_name_2 = 'cube'
    req.link_name_2 = 'base_link'
    attach_srv_a.call(req)

def deactivate_gripper():
    rospy.loginfo("Detach request received")
    attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    attach_srv_d.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = 'edrone'
    req.link_name_1 = 'Cup'
    req.model_name_2 = 'cube'
    req.link_name_2 = 'base_link'
    attach_srv_d.call(req)


# def check(self):
#     try:
#         self.box_index = self.model_state_msg.name.index(self.box_model_name)
#         self.box_coordinates[0] = self.model_state_msg.pose[self.box_index].position.x
#         self.box_coordinates[1] = self.model_state_msg.pose[self.box_index].position.y
#         self.box_coordinates[2] = self.model_state_msg.pose[self.box_index].position.z
#     except Exception as err:
#         self.box_index = -1
#     try:
#         self.drone_index = self.model_state_msg.name.index(self.drone_model_name)
#         self.drone_coordinates[0] = self.model_state_msg.pose[self.drone_index].position.x
#         self.drone_coordinates[1] = self.model_state_msg.pose[self.drone_index].position.y
#         self.drone_coordinates[2] = self.model_state_msg.pose[self.drone_index].position.z
#     except Exception as err:
#         self.drone_index = -1

#     if (self.box_index != -1 and self.drone_index !=-1 ):
#         if(abs(self.drone_coordinates[0] - self.box_coordinates[0]) < 0.1 and abs(self.drone_coordinates[1] - self.box_coordinates[1]) < 0.1 and (self.box_coordinates[2]-self.drone_coordinates[2])>0.105 and (self.box_coordinates[2]-self.drone_coordinates[2])>0):
#             self.pickable_flag = 'True'
#         else:
#             self.pickable_flag = 'False'
#     else:
#         self.pickable_flag = 'False'

#     self.check_pub.publish(self.pickable_flag)

def receive_message():

    global camera_wrt_origin
    rospy.init_node('data_accept', anonymous=True)
    # robot_grasp = robot_gripper()

    # rospy.Service('get_bool_val', GetBoolVal, callback)
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
            # rospy.Subscriber('/state', Bool, grasp_callback_state)
            # rospy.Subscriber('/tf', TFMessage, tf_callback_state)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue



if __name__ == '__main__':
    receive_message()