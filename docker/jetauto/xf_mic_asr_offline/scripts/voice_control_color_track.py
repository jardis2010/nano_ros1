#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/09/14
import json
import rospy
import signal
import jetauto_sdk.pid as pid
from jetauto_sdk import buzzer
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from xf_mic_asr_offline import voice_play
from jetauto_interfaces.msg import ColorsInfo, ColorDetect
from jetauto_interfaces.srv import SetColorDetectParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos

class VoiceControlColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.x_dis = 500
        self.center = None
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)
        self.pid = pid.PID(0.04, 0.0, 0.0)
        rospy.Subscriber('/voice_words', String, self.words_callback)
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        voice_play.play('running')
        set_servos(self.joints_pub, 1000, ((5, 500), ))
        rospy.loginfo('唤醒口令: 小幻小幻')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 追踪红色 追踪绿色 追踪蓝色 停止追踪')
        self.mecanum_pub.publish(Twist())
        self.color_track() 
 
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 20:
                self.center = data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        print('words:', words)
        if words is not None and words not in ['唤醒成功', '休眠', '失败5次', '失败10次']:
            if words == '追踪红色':
                msg_red = ColorDetect()
                msg_red.color_name = 'red'
                msg_red.detect_type = 'circle'
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red])
                if res.success:
                    voice_play.play('start_track_red')
                else:
                    voice_play.play('track_fail')
            elif words == '追踪绿色':
                msg_green = ColorDetect()
                msg_green.color_name = 'green'
                msg_green.detect_type = 'circle'
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_green])
                if res.success:
                    voice_play.play('start_track_green')
                else:
                    voice_play.play('track_fail')
            elif words == '追踪蓝色':
                msg_blue = ColorDetect()
                msg_blue.color_name = 'blue'
                msg_blue.detect_type = 'circle'
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_blue])
                if res.success:
                    voice_play.play('start_track_blue')
                else:
                    voice_play.play('track_fail')
            elif words == '停止追踪':
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([])
                if res.success:
                    voice_play.play('stop_track')
                else:
                    voice_play.play('stop_fail')
        elif words == '唤醒成功':
            voice_play.play('awake')
        elif words == '休眠':
            buzzer.on()
            rospy.sleep(0.05)
            buzzer.off()

    def color_track(self):
        while self.running:
            if self.center is not None:
                self.pid.SetPoint = self.center.width/2 
                self.pid.update(self.center.x)
                self.x_dis += self.pid.output
                if self.x_dis > 1000:
                    self.x_dis = 1000
                if self.x_dis < 0 :
                    self.x_dis = 0
                set_servos(self.joints_pub, 20, ((5, int(self.x_dis)), ))
                rospy.sleep(0.02)
            else:
                rospy.sleep(0.01)
        
        set_servos(self.joints_pub, 1000, ((5, 500), ))
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VoiceControlColorTrackNode('voice_control_color_track')
