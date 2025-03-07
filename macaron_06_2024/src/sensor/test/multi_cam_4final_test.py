#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np
import torch
from macaron_06.msg import Traffic, Delivery, obj_info
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import threading
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber 
class ObjectDetection:
    def __init__(self, webcam1_topic='/webcam1/image_raw', webcam2_topic='/webcam2/image_raw'):
        rospy.init_node("object_detection", anonymous=True)
        self.bridge = CvBridge()
        self.latest_img_webcam1 = None
        self.latest_img_webcam2 = None
        self.latest_img_time_webcam1 = None
        self.latest_img_time_webcam2 = None

        self.detect_sign = 'No detect'

        self.lock1 = threading.Lock()
        self.lock2 = threading.Lock()
        self.traffic_pub_combined = rospy.Publisher('/traffic_obj_combined', Traffic, queue_size=1)
        self.delivery_pub_combined = rospy.Publisher('/delivery_obj_combined', Delivery, queue_size=1)
        self.obj_pub_combined_img  = rospy.Publisher('/webcam_combined/image_raw', Image, queue_size=1)  # 결합된 이미지를 발행할 퍼블리셔 초기화
        # 단일 YOLOv8 모델 초기화 (교통 신호와 배달 표지판 모두 탐지 가능)
        self.WEIGHT_PATH = "/home/park/Downloads/2023_best_traffic.pt"
        self.best_model = YOLO(self.WEIGHT_PATH)
        

        # 모델을 GPU로 강제 전환 (필요한 경우 CPU로 변경 가능)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'  # GPU가 사용 가능하면 'cuda'를, 그렇지 않으면 'cpu'를 사용
        self.best_model.to(self.device)  # 모델을 선택된 디바이스로 이동

        # ROS 이미지 토픽 구독자 설정
        self.img_sub_webcam1 = Subscriber(webcam1_topic, Image)  # webcam1 이미지 토픽을 구독
        self.img_sub_webcam2 = Subscriber(webcam2_topic, Image)  # webcam2 이미지 토픽을 구독

        self.stop_event = threading.Event()  # 스레드를 제어하기 위한 이벤트 객체 생성

        # 이미지 처리 스레드 시작
        self.image_processing_thread = threading.Thread(target=self.process_images)  # 이미지 처리를 위한 스레드 생성
        self.image_processing_thread.start()  # 이미지 처리 스레드 시작

        # LiDAR와 카메라 데이터를 동기화할 ApproximateTimeSynchronizer 객체를 생성
        self.sync = ApproximateTimeSynchronizer([self.img_sub_webcam1 , self.img_sub_webcam2], queue_size=10, slop=0.1)  # 두 이미지 토픽을 동기화하는 객체 생성
        self.sync.registerCallback(self.callback)  # 동기화된 데이터에 대해 콜백 함수 등록

    def callback(self, img_msg1, img_msg2):
        # print("callback")  # 콜백 호출을 콘솔에 출력
        # self.lidar_callback(lidar_msg)  # (비활성화된) LiDAR 메시지를 처리하는 콜백 함수 호출
        self.img_callback_webcam1(img_msg1)  # webcam1 이미지 메시지를 처리하는 콜백 함수 호출
        self.img_callback_webcam2(img_msg2)  # webcam2 이미지 메시지를 처리하는 콜백 함수 호출
        # self.stamp = img_msg1.header.stamp  # (비활성화된) webcam1 이미지 타임스탬프 저장
    
    def img_callback_webcam1(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  # ROS 이미지를 OpenCV 이미지로 변환
            with self.lock1:
                self.latest_img_webcam1 = img  # 변환된 이미지를 저장
                self.latest_img_time_webcam1 = img_msg.header.stamp  # 이미지의 타임스탬프를 저장
            # rospy.loginfo("Webcam1 image received.")  # webcam1 이미지 수신 로그 출력
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam1 error: {e}")  # 변환 오류 발생 시 로그 출력

    def img_callback_webcam2(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  # ROS 이미지를 OpenCV 이미지로 변환
            with self.lock2:
                self.latest_img_webcam2 = img  # 변환된 이미지를 저장
                self.latest_img_time_webcam2 = img_msg.header.stamp  # 이미지의 타임스탬프를 저장
            # rospy.loginfo("Webcam2 image received.")  # webcam2 이미지 수신 로그 출력
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam2 error: {e}")  # 변환 오류 발생 시 로그 출력
    
    def state_callback(self, msg):
        self.detect_sign = msg.data
        # print(self.detect_sign)

    def combine_images(self, img1, img2):
        # 두 이미지를 수평으로 결합
        combined_img = np.hstack((img1, img2))
        return combined_img

    def yolo_detection_combined(self, img, img_time):
        try:
            ns_list = ['green', 'yellow', 'red', 'all_green', 'left']
            img = cv2.resize(img, (1280, 480))  # 결합된 이미지 크기 조정
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_img = input_img.astype(np.float32) / 255.0
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)
            
            # YOLO 모델을 사용하여 탐지
            results = self.best_model(input_tensor, verbose=False)
            # print( results )
            boxes = results[0].boxes.cpu().numpy().data
            
            # 탐지된 박스와 이미지 복사본 생성
            traffic_msg = Traffic()
            traffic_msg.header.stamp = img_time
            
            selected_boxes = [] 
            combined_img = img.copy()

            # 교통 신호 박스 처리
            left_boxes = [box for box in boxes if box[2] < 640 and box[4] > 0.5 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            right_boxes = [box for box in boxes if box[2] >= 640 and box[4] > 0.5 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]

            if len(left_boxes) >= 2:
                left_boxes.sort(key=lambda x: x[3])  # box[3]을 기준으로 정렬
                selected_left_box = left_boxes[0]  # 가장 작은 box[3]을 가진 박스 선택
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_left_box[0])
                detected_obj.ymin = int(selected_left_box[1])
                detected_obj.xmax = int(selected_left_box[2])
                detected_obj.ymax = int(selected_left_box[3])
                detected_obj.ns = results[0].names[int(selected_left_box[5])]
                traffic_msg.obj.append(detected_obj)
                selected_boxes.append(selected_left_box)
            
            elif len(left_boxes) == 1:
                selected_left_box = left_boxes[0]
                width = int(selected_left_box[2]) - int(selected_left_box[0])
                height = int(selected_left_box[3]) - int(selected_left_box[1])
                # print(f"넓이:{width * height}")
                if width * height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_left_box[0])
                    detected_obj.ymin = int(selected_left_box[1])
                    detected_obj.xmax = int(selected_left_box[2])
                    detected_obj.ymax = int(selected_left_box[3])
                    detected_obj.ns = results[0].names[int(selected_left_box[5])]
                    traffic_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_left_box)

            if len(right_boxes) >= 2:
                right_boxes.sort(key=lambda x: x[3])  # box[3]을 기준으로 정렬
                selected_right_box = right_boxes[0]  # 가장 작은 box[3]을 가진 박스 선택
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_right_box[0])
                detected_obj.ymin = int(selected_right_box[1])
                detected_obj.xmax = int(selected_right_box[2])
                detected_obj.ymax = int(selected_right_box[3])
                detected_obj.ns = results[0].names[int(selected_right_box[5])]
                traffic_msg.obj.append(detected_obj)
                selected_boxes.append(selected_right_box)
            
            elif len(right_boxes) == 1:
                selected_right_box = right_boxes[0]
                width = int(selected_right_box[2]) - int(selected_right_box[0])
                height = int(selected_right_box[3]) - int(selected_right_box[1])
                # print(f"넓이:{width * height}")
                if width * height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_right_box[0])
                    detected_obj.ymin = int(selected_right_box[1])
                    detected_obj.xmax = int(selected_right_box[2])
                    detected_obj.ymax = int(selected_right_box[3])
                    detected_obj.ns = results[0].names[int(selected_right_box[5])]
                    traffic_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_right_box)
            
            self.traffic_pub_combined.publish(traffic_msg)

            for box in selected_boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                label = results[0].names[int(cls)]
                cv2.rectangle(combined_img , (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)  # 빨간색으로 교통 박스 표시
                cv2.putText(combined_img , f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # 배달 표지판 박스 처리
            delivery_list = ['delivery_a1', 'delivery_a2', 'delivery_a3', 'delivery_b1', 'delivery_b2', 'delivery_b3']
            
            delivery_msg = Delivery()
            delivery_msg.header.stamp = img_time

            for box in boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                label = results[0].names[int(cls)]
                if conf > 0.5 and label in delivery_list:
                    cv2.rectangle(combined_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)  # 파란색으로 배달 박스 표시
                    cv2.putText(combined_img , f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                    detected_delivery = obj_info()
                    detected_delivery.xmin = int(xmin)
                    detected_delivery.ymin = int(ymin)
                    detected_delivery.xmax = int(xmax)
                    detected_delivery.ymax = int(ymax)
                    detected_delivery.ns = label
                    delivery_msg.obj.append(detected_delivery)

            self.delivery_pub_combined.publish(delivery_msg)
            
            imgmsg = self.bridge.cv2_to_imgmsg(img, 'bgr8')  # 결합된 이미지를 ROS 이미지 메시지로 변환
            imgmsg.header.stamp = self.latest_img_time_webcam1  # 이미지 메시지의 타임스탬프 설정
            self.obj_pub_combined_img.publish(imgmsg)  # 결합된 이미지 발행
            
            # self.visualize_results(combined_img, "combined")    

        except Exception as e:
            rospy.logerr(f"YOLO detection error: {e}")

    def process_images(self):
        while not self.stop_event.is_set():  # stop_event가 설정되지 않은 동안 반복
            try:
                latest_img_webcam1 = None
                latest_img_webcam2 = None
                latest_img_time_webcam1 = None
                latest_img_time_webcam2 = None

                with self.lock1:
                    if self.latest_img_webcam1 is not None:
                        latest_img_webcam1 = self.latest_img_webcam1.copy()  # 최신 webcam1 이미지 복사
                        latest_img_time_webcam1 = self.latest_img_time_webcam1  # 최신 webcam1 타임스탬프 복사

                with self.lock2:
                    if self.latest_img_webcam2 is not None:
                        latest_img_webcam2 = self.latest_img_webcam2.copy()  # 최신 webcam2 이미지 복사
                        latest_img_time_webcam2 = self.latest_img_time_webcam2  # 최신 webcam2 타임스탬프 복사

                if latest_img_webcam1 is not None and latest_img_webcam2 is not None:
                    combined_img = self.combine_images(latest_img_webcam1, latest_img_webcam2)  # 두 이미지를 결합
                    combined_img_time = latest_img_time_webcam1 if latest_img_time_webcam1 > latest_img_time_webcam2 else latest_img_time_webcam2  # 결합된 이미지의 타임스탬프 결정
                    self.yolo_detection_combined(combined_img, img_time=combined_img_time)  # 결합된 이미지에 대한 YOLO 탐지 수행
            except Exception as e:
                rospy.logerr(f"Error in process_images: {e}")  # 이미지 처리 중 오류 발생 시 로그 출력

    # def visualize_results(self, img, mode):
    #     try:
    #         # Add a title to the image window
    #         title = f"Detected Objects ({mode})"
    #         cv2.imshow(title, img)
    #         cv2.waitKey(1)
    #     except Exception as e:
    #         rospy.logerr(f"Visualization error: {e}")
            
    def stop(self):
        self.stop_event.set()
        self.image_processing_thread.join()

if __name__ == "__main__":
    detector = ObjectDetection()
    rospy.spin()
    detector.stop()