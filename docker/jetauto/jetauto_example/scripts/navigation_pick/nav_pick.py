#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import yaml
import rospy
import signal
import actionlib
import threading
import numpy as np
from jetauto_sdk.pid import PID
import jetauto_sdk.misc as misc
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers import bus_servo_control
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)
}

target_color = ""
linear_speed = 0
angular_speed = 0
yaw_angle = 90

linear_base_speed = 0.007
angular_base_speed = 0.03
stop_y = 110

d_y = 10
d_x = 10

yaw_pid = PID(P=0.00006, I=0, D=0.000)
#linear_pid = PID(P=0, I=0, D=0)
linear_pid = PID(P=0.001, I=0, D=0)
#angular_pid = PID(P=0, I=0, D=0)
angular_pid = PID(P=0.002, I=0, D=0)

isRunning = False
def shutdown(signum, frame):
    global isRunning

    isRunning = False
    rospy.loginfo('shutdown')
    mecnum_pub.publish(Twist())
    rospy.signal_shutdown('shutdown')

signal.signal(signal.SIGINT, shutdown)

place = False
start_place = False
start_pick = False
#重置
def reset():
    global target_color
    global linear_speed 
    global angular_speed
    global yaw_angle
    global start_pick 

    linear_speed = 0
    angular_speed = 0
    yaw_angle = 0
    target_color = ""
    start_pick = False

    linear_pid.clear()
    angular_pid.clear()
    bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 215), (3, 15), (4, 700), (5, 500)))

#设置目标颜色
def setTargetColor(target_color_new):
    global target_color
    target_color = target_color_new[0]
    
    return (True, ())

#初始化
def init():
    print('Nav Pick Init')
    reset()

def start():
    global isRunning
    isRunning = True
    print('Nav Pick Start')

def stop():
    global isRunning, target_color
    isRunning = False
    mecnum_pub.publish(Twist())
    reset()
    print('Nav Pick Stop')

#退出玩法
def exit():
    global isRunning, target_color    
    mecnum_pub.publish(Twist())
    target_color = ""
    isRunning = False
    print('Nav Pick Exit')

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 1:  # 只有在面积大于20，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()
    
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    
    return data

lab_data = get_yaml_data("/home/jetauto/jetauto_software/lab_tool/lab_config.yaml")

# 颜色识别
size = (320, 240)
def colorDetect(img):
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab, tuple(lab_data['lab']['Mono'][target_color]['min']), tuple(lab_data['lab']['Mono'][target_color]['max']))  # 对原图像和掩模进行位运算

    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
    
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
    #cv2.imshow('dilated', dilated) 
    center_x, center_y, angle = -1, -1, -1
    if areaMaxContour is not None and 300 < area_max:  # 有找到最大面积
        rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形
        angle = rect[2]
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        for j in range(4):
            box[j, 0] = int(misc.val_map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(misc.val_map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(img, [box], -1, (0,255,255), 2)#画出四个点组成的矩形
        #获取矩形的对角点
        ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        radius = abs(ptime_start_x - pt3_x)
        center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)#中心点       
        cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)#画出中心点       
        
        img_center_x = int(img.shape[:2][1]/2)  # 获取缩小图像的宽度值的一半
        img_center_y = int(img.shape[:2][0]/2)
        #cv2.line(img, (0, img_center_y + stop_y), (img_center_x, img_center_y + stop_y), (0, 255, 255), 3)#画出中心点

    return center_x, center_y, angle

nav_continue = False
def pick_thread():
    global start_pick, start_find, place
    global nav_continue
    while True:
        if start_pick:
            bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 325), (3, 300), (4, 200), (5, 500)))
            rospy.sleep(2.5)
            bus_servo_control.set_servos(joints_pub, 500, ((1, 560), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(joints_pub, 2000, ((1, 560), (2, 215), (3, 15), (4, 700), (5, 500)))
            rospy.sleep(2.5)
            start_pick = False
            start_find = False
            nav_continue = True
            #goal.header.stamp = rospy.Time.now()
            #goal_pub.publish(goal)
        elif place:
            mecnum_pub.publish(Twist())
            rospy.sleep(1)
            bus_servo_control.set_servos(joints_pub, 2000, ((1, 560), (2, 350), (3, 350), (4, 200), (5, 500)))
            rospy.sleep(2)
            bus_servo_control.set_servos(joints_pub, 500, ((1, 200), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(joints_pub, 2000, ((1, 200), (2, 215), (3, 15), (4, 700), (5, 500)))
            rospy.sleep(2)
            place = False
        else:
            rospy.sleep(0.01)

status = "find"
count_stop = 0
count_turn = 0
count_find = 0
start_find = False
# 玩法主操作
def run(usb_cam_img):
    global start_pick, d_x, d_y, start_find, start_place
    global target_color, count_stop, count_turn, count_find, status
    global linear_speed, angular_speed, yaw_angle, place

    if not isRunning or target_color == "":
        return usb_cam_img

    img_center_x = usb_cam_img.shape[:2][1]/2  # 获取缩小图像的宽度值的一半
    img_center_y = usb_cam_img.shape[:2][0]/2
    
    object_center_x, object_center_y, object_angle = colorDetect(usb_cam_img)
    if start_find or start_place:
        twist = Twist()
        if object_center_x > 0 and ((not start_pick and start_find) or start_place):
            ########电机pid处理#########
            #以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = img_center_y + stop_y
            if abs(object_center_y - (img_center_y + stop_y)) <= d_y:
                object_center_y = img_center_y + stop_y
            if status != "turn":
                linear_pid.update(object_center_y)  #更新pid
                tmp = linear_base_speed + linear_pid.output
                
                linear_speed = tmp
                if tmp > 0.15:
                    linear_speed = 0.15
                if tmp < -0.15:
                    linear_speed = -0.15
                if abs(tmp) <= 0.0075:
                    linear_speed = 0
             
            angular_pid.SetPoint = img_center_x
            if abs(object_center_x - img_center_x) <= d_x:
                object_center_x = img_center_x
            if status != "turn":
                angular_pid.update(object_center_x)  #更新pid
                tmp = angular_base_speed + angular_pid.output
                
                angular_speed = tmp
                if tmp > 1.2:
                    angular_speed = 1.2
                if tmp < -1.2:
                    angular_speed = -1.2
                if abs(tmp) <= 0.035:
                    angular_speed = 0
            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                if start_place:
                    start_place = False
                    place = True
                else:
                    count_turn += 1
                    if count_turn > 5:
                        count_turn = 5
                        status = "turn"
                        if object_angle > 25:
                            yaw_pid.SetPoint = 90
                            if abs(object_angle - 90) <= 1:
                                object_angle = 90
                        else:
                            yaw_pid.SetPoint = 0
                            if abs(object_angle - 0) <= 1:
                                object_angle = 0
                        yaw_pid.update(object_angle)  #更新pid
                        tmp = yaw_angle - yaw_pid.output
                        yaw_angle = tmp
                        if tmp > 1:
                            yaw_angle = 1
                        if tmp < -1:
                            yaw_angle = -1
                        if 90 > object_angle > 25:
                            count_stop = 0
                            twist.linear.y = 2*0.3*math.sin(yaw_angle/2)
                            twist.angular.z = -yaw_angle
                            mecnum_pub.publish(twist)
                        elif 25 >= object_angle > 0:
                            count_stop = 0
                            twist.linear.y = -2*0.3*math.sin(yaw_angle/2)
                            twist.angular.z = yaw_angle
                            mecnum_pub.publish(twist)
                        else:
                            count_stop += 1
                            if count_stop > 10:
                                d_x = 5
                                d_y = 5
                                status = "finally"
                            if count_stop > 25:
                                #target_color = ""
                                status = 'find'
                                d_y = 10
                                d_x = 10
                                count_stop = 0
                                count_turn = 0
                                start_pick = True
                            mecnum_pub.publish(Twist())
            else:
                count_turn = 0
                if status != 'turn':
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed
                    mecnum_pub.publish(twist)
        else:
            mecnum_pub.publish(twist)
    return usb_cam_img

count_frame = 0
def image_callback(ros_image):
    global count_frame 
    count_frame += 1
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
    if isRunning and count_frame >= 10:
        count_frame = 10
        result_image = run(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
    else:
        result_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    #cv2.imshow('image', result_image)
    #cv2.waitKey(1)

pick = False
def status_callback(msg):
    global start_find, start_place, target_color, stop_y, d_x, d_y, pick
    if msg.status.status == 3 and not pick:
        pick = True
        start_find = True
    elif msg.status.status == 3 and nav_continue:
        start_place = True
        d_y = 30
        d_x = 30
        stop_y = 50
        target_color = 'red'

goal = None
def nav_goal_callback(msg):
    global goal
    goal = msg

if __name__ == '__main__':
    rospy.init_node('color_track', anonymous=True)
    
    mecnum_pub = rospy.Publisher('jetauto_controller/cmd_vel', Twist, queue_size=1)
    joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    image_topic = rospy.get_param('~image_topic', '/usb_cam/image_rect_color')
    rospy.Subscriber(image_topic, Image, image_callback)
    
    cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
    nav_goal = rospy.get_param('~nav_goal', '/move_base_simple/goal')
    rospy.Subscriber(nav_goal, PoseStamped, nav_goal_callback)
    
    move_base_result = rospy.get_param('~move_base_result', '/move_base/result')
    rospy.Subscriber(move_base_result, MoveBaseActionResult, status_callback)
    goal_pub = rospy.Publisher(nav_goal, PoseStamped, queue_size=1)
    
    rospy.sleep(0.2)
    init()
    target_color = 'blue'
    mecnum_pub.publish(Twist())
    threading.Thread(target=pick_thread, daemon=True).start()
    start()
    try:
        rospy.spin()
    except Exception as e:
        mecnum_pub.publish(Twist())
        rospy.logerr(str(e))
