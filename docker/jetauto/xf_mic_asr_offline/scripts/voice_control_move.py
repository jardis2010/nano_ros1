#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/07/25
import os
import json
import math
import rospy
import signal
import numpy as np
import jetauto_sdk.pid as pid
import jetauto_sdk.misc as misc
from jetauto_sdk import buzzer
import sensor_msgs.msg as sensor_msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play

MAX_SCAN_ANGLE = 240 # 激光的扫描角度,去掉总是被遮挡的部分degree (laser scanning angle. The covered part will always be eliminated)
CAR_WIDTH = 0.4 # meter

class VoiceControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0 
        self.scan_angle = math.radians(20)
        
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback) 
        rospy.Subscriber('/voice_words', String, self.words_callback)
        rospy.Subscriber('/mic/awake/angle', Int32, self.angle_callback)
        
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.loginfo('唤醒口令: 小幻小幻')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 左转 右转 前进 后退 漂移 过来')
        
        self.time_stamp = rospy.get_time()
        self.current_time_stamp = rospy.get_time()
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        print('words:', self.words)
        if self.words is not None and self.words not in ['唤醒成功', '休眠', '失败5次', '失败10次']:
            pass
        elif self.words == '唤醒成功':
            voice_play.play('awake')
        elif self.words == '休眠':
            buzzer.on()
            rospy.sleep(0.05)
            buzzer.off()
            #voice_play.play('sleep')

    def angle_callback(self, msg):
        self.angle = msg.data + 90
        print('angle:', self.angle)
        self.start_follow = False
        #buzzer.on()
        #rospy.sleep(0.1)
        #buzzer.off()
        self.mecanum_pub.publish(Twist())

    def lidar_callback(self, lidar_data:sensor_msg.LaserScan):
        twist = Twist()
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]
        right_ranges = lidar_data.ranges[::-1][:max_index]
        if self.start_follow:
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_ranges, right_ranges = left_ranges[:angle_index], right_ranges[:angle_index]
            ranges = list(right_ranges[::-1])
            ranges.extend(left_ranges)
            min_index = np.argmin(np.array(ranges)) # 找出距离最小值(find the minimum distance)
            dist = ranges[min_index]
            angle = -angle + lidar_data.angle_increment * min_index # 计算最小值对应的角度(calculate the angle corresponding to minimum value)
            if dist < self.threshold and abs(math.degrees(angle)) > 5: # 控制左右(left and right control)
                self.pid_yaw.update(-angle)
                twist.angular.z = misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
            else:
                self.pid_yaw.clear()

            if dist < self.threshold and abs(self.stop_dist - dist) > 0.02:
                self.pid_dist.update(self.stop_dist - dist)
                twist.linear.x = misc.set_range(self.pid_dist.output, -self.speed, self.speed)
            else:
                self.pid_dist.clear()
            if abs(twist.angular.z) < 0.008: 
                twist.angular.z = 0
            if abs(twist.linear.x) < 0.05:
                twist.linear.x = 0
            if twist.linear.x == 0 and twist.angular.z == 0:
                self.count += 1
            if self.count >= 10:
                self.count = 0
                self.start_follow = False
            self.mecanum_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                twist = Twist()
                if self.words == '前进':
                    voice_play.play('go')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = 0.2
                elif self.words == '后退':
                    voice_play.play('back')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = -0.2
                elif self.words == '左转':
                    voice_play.play('turn_left')
                    self.time_stamp = rospy.get_time() + 2
                    twist.angular.z = 0.8
                elif self.words == '右转':
                    voice_play.play('turn_right')
                    self.time_stamp = rospy.get_time() + 2
                    twist.angular.z = -0.8
                elif self.words == '漂移':
                    voice_play.play('drift')
                    twist.linear.y = 0.2
                    twist.angular.z = -0.5
                    self.time_stamp = rospy.get_time() + 2*math.pi/0.5                   
                elif self.words == '过来':
                    voice_play.play('come')
                    if self.angle > 180:
                        twist.angular.z = 1
                        self.time_stamp = rospy.get_time() + math.radians(360 - self.angle)                   
                    else:
                        twist.angular.z = -1
                        self.time_stamp = rospy.get_time() + math.radians(self.angle) 
                    self.lidar_follow = True 
                elif self.words == '休眠':
                    rospy.sleep(0.01)
                self.words = None
                self.haved_stop = False
                self.mecanum_pub.publish(twist)
            else:
                rospy.sleep(0.01)
            self.current_time_stamp = rospy.get_time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop:
                self.mecanum_pub.publish(Twist())
                self.haved_stop = True
                if self.lidar_follow:
                    self.lidar_follow = False
                    self.start_follow = True

if __name__ == "__main__":
    VoiceControlNode('voice_control')
