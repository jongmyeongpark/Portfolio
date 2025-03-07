e#!/usr/bin/env python
# -*-coding:utf-8-*-

# 1. object_detection_jjj 에서 쏴준 정보를 ros 로 받음
# 2. 넓이를 기준 으로 처리 해줌 -> 원래는 xmin 을 고려 하려 했으나 양쪽에서 잘 동작할 수 있게끔 넓이로 수정
# 3. 가중치를 두어서 정확하게 처리해 줌
# 4. 멈추는 구간을 return 함 -> state_jjj 에서 구간을 받아 미션 처리

# Python packages
import rospy
import sys, os
import numpy as np

from macaron_5.msg import Traffic, obj_info

def obj_callback(self, data):
    boxes = list()
    for sign in data.obj:
        if sign.ns in self.sign_list:  # 표지판 인 경우만 detect
            # 넓이와 class 이름 tuple 형태로 추가
            boxes.append(((abs(sign.xmin - sign.xmax)) * (abs(sign.ymin - sign.ymax)), sign.ns))
    print(boxes)

if __name__ == '__main__':
    rospy.init_node("jjjjj") # 노드 생성
    sub_sign = rospy.Subscriber('/traffic_obj', Traffic, obj_callback, queue_size=1)

    rospy.spin()
    #sdfsdf

