#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/13
import os

wav_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'feedback_voice')

def get_path(f, english=False):
    if english:
        return os.path.join(wav_path, 'english', f + '.wav')
    else:
        return os.path.join(wav_path, f + '.wav')

def play(voice, volume=100, english=False):
    try:
        res = os.popen('pactl list short sinks | grep \'USB_PnP_Audio_Device\'')
        index = res.readlines()
        device_index = index[0].split()[0]
        #print(device_index)
        os.system('pactl set-sink-volume {} {}%'.format(device_index, volume))
        res = os.popen('aplay -l | grep \'USB PnP Audio Device\'')
        index = res.readlines()
        hw = index[0].split(':')[0].split()[-1]
        #print(hw)
        os.system('aplay -q -D \"plug:SLAVE=\'hw:%s,0\'\" '%hw + get_path(voice, english))
    except BaseException as e:
        print(e)

if __name__ == '__main__':
    play('ok')
