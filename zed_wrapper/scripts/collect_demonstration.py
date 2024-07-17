#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
bridge = CvBridge()
import message_filters

from pathlib import Path

from datetime import datetime

import os
import re

def get_next_image_filename(directory, prefix='Image', extension='.png'):
    # List all files in the directory
    files = os.listdir(directory)
    
    # Initialize a list to hold the numbers
    numbers = []
    
    # Regular expression to find the pattern 'Image<number>.png'
    pattern = re.compile(f'^{prefix}(\\d+){extension}$')
    
    for filename in files:
        match = pattern.match(filename)
        if match:
            # Extract the number part and convert to an integer
            numbers.append(int(match.group(1)))
    
    # Find the highest number
    if numbers:
        next_number = max(numbers) + 1
    else:
        next_number = 1
    
    # Create the next filename
    next_filename = f"{prefix}{next_number}{extension}"
    return next_filename


def create_callback(filepath):
    def callback(image0_l, image0_r, image1_l, image1_r, image2_l, image2_r, control):
        cv_image0_l = bridge.imgmsg_to_cv2(image0_l, desired_encoding='passthrough')
        cv_image1_l = bridge.imgmsg_to_cv2(image1_l, desired_encoding='passthrough')
        cv_image2_l = bridge.imgmsg_to_cv2(image2_l, desired_encoding='passthrough') 
        cv_image0_r = bridge.imgmsg_to_cv2(image0_r, desired_encoding='passthrough')
        cv_image1_r = bridge.imgmsg_to_cv2(image1_r, desired_encoding='passthrough')
        cv_image2_r = bridge.imgmsg_to_cv2(image2_r, desired_encoding='passthrough') 


        fname = get_next_image_filename(file_path + "/cam0_left/")
        cv2.imwrite(filepath + "/cam0_left/"+ fname, cv_image0_l)
        fname = get_next_image_filename(file_path + "/cam1_left/")
        cv2.imwrite(filepath + "/cam1_left/"+ fname, cv_image1_l)
        fname = get_next_image_filename(file_path + "/mini_left/")
        cv2.imwrite(filepath + "/mini_left/"+ fname, cv_image2_l)

        fname = get_next_image_filename(file_path + "/cam0_right/")
        cv2.imwrite(filepath + "/cam0_right/"+ fname, cv_image0_r)
        fname = get_next_image_filename(file_path + "/cam1_right/")
        cv2.imwrite(filepath + "/cam1_right/"+ fname, cv_image1_r)
        fname = get_next_image_filename(file_path + "/mini_right/")
        cv2.imwrite(filepath + "/mini_right/"+ fname, cv_image2_r)

    return callback





# def listener():
#      rospy.init_node('listener', anonymous=True)
 
#      rospy.Subscriber("/zed2/zed_node/right_raw/image_raw_color", Image, save_image)
 
#      # spin() simply keeps python from exiting until this node is stopped
#      rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    file_path = rospy.get_param('~file_path', '/home/necl/panda/data/')
    file_path = file_path + "/" + datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    os.makedirs(file_path + "/cam0_left")
    os.makedirs(file_path + "/cam0_right")
    os.makedirs(file_path + "/cam1_left")
    os.makedirs(file_path + "/cam1_right")
    os.makedirs(file_path + "/mini_left")
    os.makedirs(file_path + "/mini_right")

    image0_left_sub = message_filters.Subscriber('/zed2_0/stereo_cam/left_raw/image_raw_color', Image)
    image0_right_sub = message_filters.Subscriber('/zed2_0/stereo_cam/right_raw/image_raw_color', Image)
    image1_left_sub = message_filters.Subscriber('/zed2_1/stereo_cam/left_raw/image_raw_color', Image)
    image1_right_sub = message_filters.Subscriber('/zed2_1/stereo_cam/right_raw/image_raw_color', Image)
    image2_left_sub = message_filters.Subscriber('/zed2_mini/stereo_cam/left_raw/image_raw_color', Image)
    image2_right_sub = message_filters.Subscriber('/zed2_mini/stereo_cam/right_raw/image_raw_color', Image)
    control_sub = message_filters.Subscriber('/robot_info', String)

    ts = message_filters.ApproximateTimeSynchronizer([image0_left_sub, 
                                                      image0_right_sub,
                                                      image1_left_sub, 
                                                      image1_right_sub,
                                                      image2_left_sub, 
                                                      image2_right_sub,
                                                      control_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(create_callback(str(file_path)+"/"))
    rospy.spin()