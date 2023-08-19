#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/12
# 自检程序(self-test program)
import sys
import time
import smbus2
from jetauto_sdk import buzzer, button
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

def check_sensor():
    bus = smbus2.SMBus(1)
    count_imu = 0
    while True:
        count_imu += 1
        try:
            bus.write_byte_data(0x68, 0x6b, 0)
            if button.get_button_status('key1') == 1 and button.get_button_status('key2') == 1:
                buzzer.on()
                time.sleep(0.1)
                buzzer.off()
                print('everything is ok!')
            break
        except Exception as e:
            print(str(e))
        if count_imu > 100:
            count_imu = 0 
            print('imu init timeout')
            break
        time.sleep(0.01)

if __name__ == '__main__':
    controller.runAction('startup_init')
    time.sleep(2)
    check_sensor()
