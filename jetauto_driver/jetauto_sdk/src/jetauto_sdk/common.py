#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/13
import math
import yaml
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

range_rgb = {
    'red': (255, 50, 0),
    'blue': (0, 50, 255),
    'green': (50, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def cv2_image2ros(image, frame_id=''):
    ros_image = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = frame_id
    ros_image.height = image.shape[:2][0]
    ros_image.width = image.shape[:2][1]
    ros_image.encoding = 'rgb8'
    ros_image.data = image.tostring()
    ros_image.header = header
    ros_image.step = ros_image.width*3
    
    return ros_image

def get_area_max_contour(contours, threshold=100):
    '''
    获取最大面积对应的轮廓
    :param contours:
    :param threshold:
    :return:
    '''
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()
