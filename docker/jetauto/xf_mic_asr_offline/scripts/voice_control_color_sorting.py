#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/09/14
import json
import rospy
from jetauto_sdk import buzzer
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xf_mic_asr_offline import voice_play

class VoiceControlColorSortingNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.running = True
        
        rospy.Subscriber('/voice_words', String, self.words_callback)
        voice_play.play('running')
       
        rospy.loginfo('唤醒口令: 小幻小幻')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 开启颜色分拣 关闭颜色分拣')
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))
            rospy.loginfo("Shutting down")

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        print('words:', words)
        if words is not None and words not in ['唤醒成功', '休眠', '失败5次', '失败10次']:
            if words == '开启颜色分拣':
                res = rospy.ServiceProxy('/color_sorting/start', Trigger)()
                if res.success:
                    voice_play.play('open_success')
                else:
                    voice_play.play('open_fail')
            elif words == '关闭颜色分拣':
                res = rospy.ServiceProxy('/color_sorting/stop', Trigger)()
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
    VoiceControlColorSortingNode('voice_control_colorsorting')
