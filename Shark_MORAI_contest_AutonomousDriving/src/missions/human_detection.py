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


PT_PATH = "yolov8n.pt"
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

MODEL = YOLO(PT_PATH).to(DEVICE)

class shark_human:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.sub_img = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_callback, queue_size=1)
        self.human_pub = rospy.Publisher('/human', Bool, queue_size=1)
        self.img = None

        self.model = YOLO(PT_PATH) 
        self.model.to(DEVICE)
        self.human_detect_cnt = 0
        self.human_list = []
        self.dynamic_thres = 0

    def img_callback(self, img):
        self.img = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")

    def detect_human(self):
        results = self.model(self.img,
                             verbose=False)
        
        confs = results[0].boxes.conf
        boxes = results[0].boxes
        annotated_frame = self.img.copy()  

        bbox = []
        for i, box in enumerate(boxes):
            class_id = int(box.cls[0].item())
            conf = confs[i].item()  
            box_coords = box.xyxy[0].cpu().numpy()  
            area = int((box_coords[2]-box_coords[0]) * (box_coords[3]-box_coords[1])) 
            
            bbox.append(box_coords[0])
            if class_id == 0 and conf > 0.7 and area >= 2000:
                bbox.sort(reverse=True)
                self.human_detect_cnt += 1
                if self.human_detect_cnt >= 3:
                    self.human_list.append([i, bbox[0]])
                    self.human_pub.publish(True)
                    self.human_detect_cnt = 0

                unique_x = list(set([row[0] for row in self.human_list]))
                
                # print(len(self.human_list))
                if len(self.human_list) >= 2:
                    self.dynamic_thres += 1
                    for i in range(len(unique_x)):
                        if self.human_list[i][0] == i:
                            diff = abs(self.human_list[-1][1] - self.human_list[-2][1])
                            if diff >= 30:
                                print("Detect Dynamic")
                            else:
                                print("Static")
                
                if len(self.human_list) >= 300:
                    self.human_list = []

                # print()
                # print(self.human_list)
                # print()
                cv2.rectangle(annotated_frame, 
                            (int(box_coords[0]), int(box_coords[1])), 
                            (int(box_coords[2]), int(box_coords[3])), 
                            (0, 255, 0), 2)  
                
                label = f'Pedestrian: {conf:.2f}'
                cv2.putText(annotated_frame, label, 
                            (int(box_coords[0]), int(box_coords[1]) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (0, 255, 0), 2)
            
            

        cv2.imshow('frame', annotated_frame)
        cv2.waitKey(1)

class YoloPersonDetect:
    def __init__(self):
        self.bridge = CvBridge()
        
        # self.sub_img = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_callback, queue_size=1)
        # self.human_pub = rospy.Publisher('/human', Bool, queue_size=1)

        self.model = YOLO(PT_PATH) 
        self.model.to(DEVICE)
        self.human_detect_cnt = 0


    # def img_callback(self, img):
    #     print("Image received!")
    #     self.img = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")

    def detect_human(self, img):
        results = self.model(img,
                             verbose=False)
        
        confs = results[0].boxes.conf
        boxes = results[0].boxes
        # annotated_frame = self.img.copy()  

        for i, box in enumerate(boxes):
            class_id = int(box.cls[0].item())
            conf = confs[i].item()  
            box_coords = box.xyxy[0].cpu().numpy()   # x, y
            area = int((box_coords[2]-box_coords[0]) * (box_coords[3]-box_coords[1])) 
            
            # print('area: ', area)
            if class_id == 0 and conf > 0.7 and area >= 2000:
                self.human_detect_cnt += 1
                if self.human_detect_cnt >= 3:
                    self.human_detect_cnt = 0
                    return True
                
        return False

                # cv2.rectangle(annotated_frame, 
                #             (int(box_coords[0]), int(box_coords[1])), 
                #             (int(box_coords[2]), int(box_coords[3])), 
                #             (0, 255, 0), 2)  
                
                # label = f'Pedestrian: {conf:.2f}'
                # cv2.putText(annotated_frame, label, 
                #             (int(box_coords[0]), int(box_coords[1]) - 10), 
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                #             (0, 255, 0), 2)
            


        # cv2.imshow('frame', annotated_frame)
        # cv2.waitKey(1)

def main():
    rospy.init_node('person_detect')
    human_detection = shark_human() 

    while not rospy.is_shutdown():
        human_detection.detect_human()

if __name__ == '__main__':
    main()
