#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/09/14
import json
import rospy
import signal
from jetauto_sdk import buzzer
from std_msgs.msg import String
from xf_mic_asr_offline import voice_play
from jetauto_interfaces.msg import ColorsInfo, ColorDetect
from jetauto_interfaces.srv import SetColorDetectParam

class VoiceControlColorDetectNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.count = 0
        self.color = None
        self.running = True
        self.last_color = None 
        signal.signal(signal.SIGINT, self.shutdown)
        
        rospy.Subscriber('/voice_words', String, self.words_callback)
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        voice_play.play('running')
       
        rospy.loginfo('唤醒口令: 小幻小幻()')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 开启颜色识别 关闭颜色识别')

        while self.running:
            if self.color == 'red' and self.last_color != 'red':
                self.last_color = 'red'
                voice_play.play('red')
            elif self.color == 'green' and self.last_color != 'green':
                self.last_color = 'green'
                voice_play.play('green')
            elif self.color == 'blue' and self.last_color != 'blue':
                self.last_color = 'blue'
                voice_play.play('blue')
            else:
                self.count += 1
                rospy.sleep(0.01)
                if self.count > 50:
                    self.count = 0
                    self.last_color = self.color
        rospy.signal_shutdown('shutdown')
 
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 30:
                self.color = data[0].color
            else:
                self.color = None
        else:
            self.color = None

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        print('words:', words)
        if words is not None and words not in ['唤醒成功', '休眠', '失败5次', '失败10次']:
            if words == '开启颜色识别':
                msg_red = ColorDetect()
                msg_red.color_name = 'red'
                msg_red.detect_type = 'circle'
                msg_green = ColorDetect()
                msg_green.color_name = 'green'
                msg_green.detect_type = 'circle'
                msg_blue = ColorDetect()
                msg_blue.color_name = 'blue'
                msg_blue.detect_type = 'circle'
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red, msg_green, msg_blue])
                if res.success:
                    voice_play.play('open_success')
                else:
                    voice_play.play('open_fail')
            elif words == '关闭颜色识别':
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([])
                if res.success:
                    voice_play.play('close_success')
                else:
                    voice_play.play('close_fail')
        elif words == '唤醒成功':
            voice_play.play('awake')
        elif words == '休眠':
            buzzer.on()
            rospy.sleep(0.05)
            buzzer.off()

if __name__ == "__main__":
    VoiceControlColorDetectNode('voice_control_color_detect')
