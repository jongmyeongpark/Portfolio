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
        # YOLOv8 모델 초기화
        self.WEIGHT_PATH = "/home/wt/다운로드/pt/2023_best_traffic.pt"
        self.WEIGHT_PATH2 = "/home/wt/다운로드/delievery_yolov8s.pt"
        
        self.traffic_model = YOLO(self.WEIGHT_PATH)
        self.delivery_model = YOLO(self.WEIGHT_PATH2)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.traffic_model.to(self.device)
        self.delivery_model.to(self.device)

        # ROS 이미지 토픽 구독자 설정
        self.img_sub_webcam1 = rospy.Subscriber(webcam1_topic, Image, self.img_callback_webcam1, queue_size=1)
        self.img_sub_webcam2 = rospy.Subscriber(webcam2_topic, Image, self.img_callback_webcam2, queue_size=1)
        self.sub_sign_state = rospy.Subscriber('/sign', String, self.state_callback, queue_size=1)

        self.stop_event = threading.Event()
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def img_callback_webcam1(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock1:
                self.latest_img_webcam1 = img
                self.latest_img_time_webcam1 = img_msg.header.stamp
            rospy.loginfo("Webcam1 image received.")
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam1 error: {e}")

    def img_callback_webcam2(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock2:
                self.latest_img_webcam2 = img
                self.latest_img_time_webcam2 = img_msg.header.stamp
            rospy.loginfo("Webcam2 image received.")
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam2 error: {e}")

    def state_callback(self, msg):
        self.detect_sign = msg.data
        print(self.detect_sign)

    def combine_images(self, img1, img2):
        # Stack images horizontally
        combined_img = np.hstack((img1, img2))
        return combined_img

    def yolo_detection_combined(self, img, img_time):
        try:
            ns_list = ['green', 'yellow', 'red', 'all_green', 'left']
            img = cv2.resize(img, (1280, 480))  # Resize to fit the combined image width
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_img = input_img.astype(np.float32) / 255.0
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)
            
            # Traffic model detection
            results_traffic = self.traffic_model(input_tensor, verbose=False)
            boxes_traffic = results_traffic[0].boxes.cpu().numpy().data
            
            # Delivery model detection
            results_delivery = self.delivery_model(input_tensor, verbose=False)
            boxes_delivery = results_delivery[0].boxes.cpu().numpy().data

            traffic_msg = Traffic()
            traffic_msg.header.stamp = img_time
            
            selected_boxes = [] 
            traffic_img = img.copy()
            delivery_img = img.copy()

            # Process traffic boxes
            left_boxes = [box for box in boxes_traffic if box[2] < 640 and box[4] > 0.5 and results_traffic[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            right_boxes = [box for box in boxes_traffic if box[2] >= 640 and box[4] > 0.5 and results_traffic[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]

            # # Visualize traffic boxes
            # for box in left_boxes + right_boxes:
            #     xmin, ymin, xmax, ymax, conf, cls = box
            #     label = results_traffic[0].names[int(cls)]
            #     if label in ns_list:
            #         cv2.rectangle(traffic_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)  # Red color for traffic boxes
            #         cv2.putText(traffic_img, f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            if len(left_boxes) >= 2:
                left_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_left_box = left_boxes[0]  # Select the box with the smallest box[3]
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_left_box[0])
                detected_obj.ymin = int(selected_left_box[1])
                detected_obj.xmax = int(selected_left_box[2])
                detected_obj.ymax = int(selected_left_box[3])
                detected_obj.ns = results_traffic[0].names[int(selected_left_box[5])]
                traffic_msg.obj.append(detected_obj)
                selected_boxes.append(selected_left_box)
            
            elif len(left_boxes) == 1:
                selected_left_box = left_boxes[0]
                width = int(selected_left_box[2]) - int(selected_left_box[0])
                height = int(selected_left_box[3]) - int(selected_left_box[1])
                print(f"넓이:{width * height}")
                if width * height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_left_box[0])
                    detected_obj.ymin = int(selected_left_box[1])
                    detected_obj.xmax = int(selected_left_box[2])
                    detected_obj.ymax = int(selected_left_box[3])
                    detected_obj.ns = results_traffic[0].names[int(selected_left_box[5])]
                    traffic_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_left_box)

            if len(right_boxes) >= 2:
                right_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_right_box = right_boxes[0]  # Select the box with the smallest box[3]
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_right_box[0])
                detected_obj.ymin = int(selected_right_box[1])
                detected_obj.xmax = int(selected_right_box[2])
                detected_obj.ymax = int(selected_right_box[3])
                detected_obj.ns = results_traffic[0].names[int(selected_right_box[5])]
                traffic_msg.obj.append(detected_obj)
                selected_boxes.append(selected_right_box)
            
            elif len(right_boxes) == 1:
                selected_right_box = right_boxes[0]
                width = int(selected_right_box[2]) - int(selected_right_box[0])
                height = int(selected_right_box[3]) - int(selected_right_box[1])
                print(f"넓이:{width * height}")
                if width * height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_right_box[0])
                    detected_obj.ymin = int(selected_right_box[1])
                    detected_obj.xmax = int(selected_right_box[2])
                    detected_obj.ymax = int(selected_right_box[3])
                    detected_obj.ns = results_traffic[0].names[int(selected_right_box[5])]
                    traffic_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_right_box)
            
            self.traffic_pub_combined.publish(traffic_msg)

            for box in selected_boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                label = results_traffic[0].names[int(cls)]
                cv2.rectangle(traffic_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)  # Red color for traffic boxes
                cv2.putText(traffic_img, f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # Process delivery boxes
            delivery_list = ['delivery_a1', 'delivery_a2', 'delivery_a3', 'delivery_b1', 'delivery_b2', 'delivery_b3']
            
            delivery_msg = Delivery()
            delivery_msg.header.stamp = img_time
            # Visualize delivery boxes
            for box in boxes_delivery:
                xmin, ymin, xmax, ymax, conf, cls = box
                label = results_delivery[0].names[int(cls)]
                if conf > 0 and label in delivery_list:
                    cv2.rectangle(delivery_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)  # Green color for delivery boxes
                    cv2.putText(delivery_img, f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    detected_obj = obj_info()
                    detected_obj.xmin = int(xmin)
                    detected_obj.ymin = int(ymin)
                    detected_obj.xmax = int(xmax)
                    detected_obj.ymax = int(ymax)
                    detected_obj.ns = label
                    delivery_msg.obj.append(detected_obj)
            
            self.delivery_pub_combined.publish(delivery_msg)        
            # Combine images
            combined_img = np.vstack((traffic_img, delivery_img))
                      
            self.visualize_results(combined_img, "combined")
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

    def visualize_results(self, img, mode):
        try:
            # Add a title to the image window
            title = f"Detected Objects ({mode})"
            cv2.imshow(title, img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Visualization error: {e}")

if __name__ == "__main__":
    try:
        obj_detection = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
