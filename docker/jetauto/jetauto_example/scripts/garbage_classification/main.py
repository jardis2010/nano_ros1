#!/usr/bin/env python3
# encoding: utf-8
# 巡线
import sys
import cv2
import math
import rospy
import signal
import threading
import numpy as np
from sensor_msgs.msg import Image
import geometry_msgs.msg as geo_msg
from jetauto_interfaces.msg import ObjectsInfo
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}

class GarbageClassificationNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.running = True
        self.image = None
        self.center = None
        self.count = 0
        self.class_name = None
        self.start_pick = False
        self.current_class_name = None
        self.pick_roi = [160, 190, 305, 335] #[y_min, y_max, x_min, x_max]
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', geo_msg.Twist, queue_size=1)  # 底盘控制
        rospy.Subscriber('/yolov5/object_image', Image, self.image_callback)  # 摄像头订阅
        rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
        rospy.sleep(0.2)
        signal.signal(signal.SIGINT, self.shutdown)
        controller.runAction('garbage_pick_init')
        threading.Thread(target=self.pick, daemon=True).start()
        self.mecanum_pub.publish(geo_msg.Twist())
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    def pick(self):
        while True:
            waste_category = None
            if self.start_pick:
                rospy.sleep(0.2)
                for k, v in WASTE_CLASSES.items():
                    if self.current_class_name in v:
                        waste_category = k
                        break
                self.class_name = None
                print(waste_category)
                controller.runAction('garbage_pick')
                if waste_category == 'food_waste':
                    controller.runAction('place_food_waste')
                elif waste_category == 'hazardous_waste':
                    controller.runAction('place_hazardous_waste')
                elif waste_category == 'recyclable_waste':
                    controller.runAction('place_recyclable_waste')
                elif waste_category == 'residual_waste':
                    controller.runAction('place_residual_waste')
                self.start_pick = False
            else:
                rospy.sleep(0.01)

    def image_proc(self):
        while self.running:
            if self.image is not None:
                if not self.start_pick:
                    cv2.rectangle(self.image, (self.pick_roi[2] - 30, self.pick_roi[0] - 30), (self.pick_roi[3] + 30, self.pick_roi[1] + 30), (0, 255, 255), 2)
                cv2.imshow('image', self.image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.running = False
            if self.class_name is not None and not self.start_pick:
                self.count += 1
                if self.count > 50:
                    self.current_class_name = self.class_name
                    self.start_pick = True
                    self.count = 0
            else:
                self.count = 0
                rospy.sleep(0.01)
        self.mecanum_pub.publish(geo_msg.Twist())
        controller.runAction('garbage_pick_init')
        rospy.signal_shutdown('shutdown')

    def get_object_callback(self, msg):
        objects = msg.objects
        if objects == []:
            self.center = None
            self.class_name = None
        else:
            for i in objects:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                if self.pick_roi[2] < center[0] < self.pick_roi[3] and self.pick_roi[0] < center[1] < self.pick_roi[1]:
                    self.center = center
                    self.class_name = class_name

if __name__ == "__main__":
    GarbageClassificationNode('garbage_classification')
