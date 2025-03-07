#!/usr/bin/env python3
# -*-coding:utf-8-*-

import rospy
import cv2
import numpy as np
import torch

from enum import IntEnum
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

'''Publish 정보
True: Stop
False: Go

'''


PT_PATH = "/home/pc/catkin_ws/src/morai_2/src/missions/traffic_light.pt"
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

MODEL = YOLO(PT_PATH).to(DEVICE)
print(MODEL.names)

class traffic_info(IntEnum):
    red = 0 # False
    yellow = 1 # False
    otherwise = 2  # True


class shark_traffic_light_detect:
    def __init__(self):
        self.bridge = CvBridge()
        
        # self.sub_img = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_callback, queue_size=1)
        self.traffic_light_pub = rospy.Publisher('/stop', Bool, queue_size=1)
        self.img = None
        self.img_flag = False

        self.model = YOLO(PT_PATH) 
        self.model.to(DEVICE)

    # def img_callback(self, img):
    #     if not self.img_flag:
    #         print("Image received!")
    #         self.img = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")
    #         self.img_flag = True  

    def detect_traffic_light(self, img, current_s):
        results = self.model(img)
        annotated_frame = results[0].plot()

        class_id = 2
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())  # 0: 'green', 1: 'yellow', 2: 'red', 3: 'all_green', 4: 'left'
        
        # Stop
        if (class_id == 1 or class_id == 2) and current_s >= 681:
            self.traffic_light_pub.publish(True)
        
        # Go
        
        # cv2.imshow('frame', annotated_frame)
        # cv2.waitKey(1)

def main():
    rospy.init_node('traffic_light')
    traffic_light_detection = shark_traffic_light_detect() 

    while not rospy.is_shutdown():
        traffic_light_detection.detect_traffic_light()

if __name__ == '__main__':
    main()
