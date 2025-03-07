#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np
import torch
from macaron_06.msg import Cone, obj_info
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from ultralytics import YOLO

# 1,2,3,4 , 6,7,8,9
# def find_camera_index(min_index=0, max_index=15):
#     camera_indexes = []
#     for i in range(min_index, max_index):
#         cap = cv2.VideoCapture(i)
#         if cap.isOpened():
#             camera_indexes.append(i)
#     return camera_indexes[0], camera_indexes[2]
# a, b = find_camera_index()

# print('a, b :', a, b)

class ObjectDetection:
    def __init__(self, webcam1_id=0, webcam2_id=4):
        rospy.init_node("object_detection", anonymous=True)
        self.bridge = CvBridge()

        # YOLO 모델 로드
        self.WEIGHT_PATH = "/home/macaron/catkin_ws/src/macaron_06/src/sensor/pre/tracking/traffic_cone_yolov8s.pt"
        self.model = YOLO(self.WEIGHT_PATH)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        # OpenCV 웹캠 설정
        self.cap1 = cv2.VideoCapture(webcam1_id)
        self.cap2 = cv2.VideoCapture(webcam2_id)
        cv2.waitKey(1)
        # 결합된 이미지와 탐지 결과 발행
        self.obj_pub_combined = rospy.Publisher('cone_obj_combined', Cone, queue_size=1)
        self.obj_pub_combined_img = rospy.Publisher('/webcam_combined/image_raw', Image, queue_size=1)

        self.stop_event = threading.Event()
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def combine_images(self, img1, img2):
        # 두 웹캠 이미지를 수평으로 결합
        combined_img = np.hstack((img1, img2))
        return combined_img

    def yolo_detection_combined(self, img, img_time):
        try:
            # 탐지할 객체 리스트 (파란색, 노란색 콘)
            ns_list = ['blue_cone', 'yellow_cone']

            # 이미지를 YOLO 입력에 맞게 전처리
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_img = input_img.astype(np.float32) / 255.0
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)
            
            # YOLO 모델로 추론 수행
            results = self.model(input_tensor, verbose=False)
            boxes = results[0].boxes.cpu().numpy().data

            # 탐지 결과 메시지 생성
            obj_msg = Cone()
            obj_msg.header.stamp = img_time
            for box in boxes:
                label = results[0].names[int(box[5])]
                if box[4] > 0.2 and label in ns_list:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(box[0])
                    detected_obj.ymin = int(box[1])
                    detected_obj.xmax = int(box[2])
                    detected_obj.ymax = int(box[3])
                    detected_obj.ns = label
                    obj_msg.obj.append(detected_obj)
            
            # 탐지 결과와 결합된 이미지 발행
            self.obj_pub_combined.publish(obj_msg)
            imgmsg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            imgmsg.header.stamp = img_time
            self.obj_pub_combined_img.publish(imgmsg)

            # YOLO 탐지 결과를 시각화
            for box in boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                if int(cls) < len(ns_list):
                    # 바운딩 박스 그리기
                    cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                    # 클래스 이름을 바운딩 박스 위에 표시
                    cv2.putText(img, ns_list[int(cls)], (int(xmin), int(ymin) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    # 신뢰도 점수를 클래스 이름 위에 표시
                    cv2.putText(img, f"{conf:.2f}", (int(xmin), int(ymin) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # 결과 이미지 표시
            cv2.imshow("YOLO Detection", img)
            cv2.waitKey(1)  # 키 입력 대기

        except Exception as e:
            rospy.logerr(f"YOLO detection error: {e}")

    def process_images(self):
        while not self.stop_event.is_set():
            try:
                # 두 개의 웹캠에서 이미지를 읽어옴
                ret1, frame1 = self.cap1.read()
                ret2, frame2 = self.cap2.read()

                if ret1 and ret2:
                    # 결합된 이미지를 생성
                    combined_img = self.combine_images(frame1, frame2)

                    # 현재 시간을 가져옴
                    img_time = rospy.Time.now()

                    # YOLO 객체 탐지 수행
                    self.yolo_detection_combined(combined_img, img_time=img_time)

            except Exception as e:
                rospy.logerr(f"Error in process_images: {e}")

    def cleanup(self):
        # 프로그램 종료 시 스레드 중지 및 웹캠 릴리즈
        self.stop_event.set()
        self.image_processing_thread.join()
        self.cap1.release()
        self.cap2.release()
        rospy.signal_shutdown("Shutting down")


def main():
    ob = None  # ob 변수를 미리 선언해 둡니다.
    try:
        ob = ObjectDetection()  # 두 웹캠을 OpenCV로 사용하여 탐지
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if ob is not None:  # ob가 None이 아닌 경우에만 cleanup() 호출
            ob.cleanup()

if __name__ == "__main__":
    main()
