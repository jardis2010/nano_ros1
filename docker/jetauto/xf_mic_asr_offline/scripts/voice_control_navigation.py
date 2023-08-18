#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/07/25
import json
import rospy
import signal
from jetauto_sdk import buzzer
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        
        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.last_status = Twist()
        
        self.mecanum_pub = rospy.Publisher('/jetauto_1/jetauto_controller/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/jetauto_1/move_base_simple/goal', PoseStamped, queue_size=1)
        self.goal_status_pub = rospy.Publisher('/jetauto_1/move_base/result', MoveBaseActionResult, queue_size=1)
        rospy.Subscriber('/voice_words', String, self.words_callback)
        rospy.Subscriber('/mic/awake/angle', Int32, self.angle_callback)
        
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.loginfo('唤醒口令: 小幻小幻')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 去A点 去B点 去C点 回原点')
        
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
        self.angle = msg.data
        print('angle:', self.angle)
        #buzzer.on()
        #rospy.sleep(0.1)
        #buzzer.off()

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                pose = PoseStamped()
                pose.header.frame_id = 'jetauto_1/map'
                pose.header.stamp = rospy.Time.now()
                if self.words == '去A点':
                    pose.pose.position.x = 2.64
                    pose.pose.position.y = -2
                    pose.pose.orientation.w = 1
                    voice_play.play('go_a')
                    self.goal_pub.publish(pose)
                elif self.words == '去B点':
                    pose.pose.position.x = 1.25
                    pose.pose.position.y = 0.5
                    pose.pose.orientation.w = 1
                    voice_play.play('go_b')
                    self.goal_pub.publish(pose)  
                elif self.words == '去C点':
                    pose.pose.position.x = 1.4
                    pose.pose.position.y = -2.4
                    pose.pose.orientation.w = 1
                    voice_play.play('go_c')
                    self.goal_pub.publish(pose)
                elif self.words == '回原点':
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                    pose.pose.orientation.w = 1
                    voice_play.play('go_origin')
                    self.goal_pub.publish(pose)  
                elif self.words == '休眠':
                    rospy.sleep(0.01)
            else:
                rospy.sleep(0.01)

if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
