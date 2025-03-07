#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np
import torch
from macaron_06.msg import Traffic, obj_info, Delivery
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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


class DualWebcamObjectDetectionROS:
    def __init__(self, webcam1_id=0, webcam2_id=4):
        rospy.init_node("object_detection", anonymous=True)
        self.bridge = CvBridge()

        # OpenCV 웹캠 설정
        self.cap1 = cv2.VideoCapture(webcam1_id)
        cv2.waitKey(1)
        self.cap2 = cv2.VideoCapture(webcam2_id)
        cv2.waitKey(1)

        # Check if webcams are opened successfully
        if not self.cap1.isOpened() or not self.cap2.isOpened():
            rospy.logerr("Failed to open one or both webcams.")
            rospy.signal_shutdown("Webcam error")

        # YOLO 모델 로드
        self.WEIGHT_PATH = "/home/macaron/pt/2023_best_traffic.pt"
        self.model = YOLO(self.WEIGHT_PATH)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        self.latest_img_webcam1 = None
        self.latest_img_webcam2 = None
        self.latest_img_time_webcam1 = None
        self.latest_img_time_webcam2 = None

        # Initialize locks for thread safety
        self.lock1 = threading.Lock()
        self.lock2 = threading.Lock()

        # Publishers
        self.obj_pub_combined = rospy.Publisher('/traffic_obj_combined', Traffic, queue_size=1)
        self.delivery_pub_combined = rospy.Publisher('/delivery_obj_combined', Delivery, queue_size=1)
        # self.obj_pub_combined_img = rospy.Publisher('/webcam_combined/image_raw', Image, queue_size=1)

        self.stop_event = threading.Event()
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def combine_images(self, img1, img2):
        # Stack images horizontally
        combined_img = np.hstack((img1, img2))
        return combined_img

    def yolo_detection_combined(self, img, img_time):
        try:
            ns_list = ['green', 'yellow', 'red', 'all_green', 'left']
            img = cv2.resize(img, (1280, 480))  # Resize combined image
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_img = input_img.astype(np.float32) / 255.0
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)

            # YOLO 모델을 사용하여 탐지
            results = self.model(input_tensor, verbose=False)
            boxes = results[0].boxes.cpu().numpy().data

            traffic_msg = Traffic()
            traffic_msg.header.stamp = img_time

            selected_boxes = [] 
            combined_img = img.copy()

            # Process traffic signal boxes
            left_boxes = [box for box in boxes if box[2] < 640 and box[4] > 0.5 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            right_boxes = [box for box in boxes if box[2] >= 640 and box[4] > 0.5 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            if len(left_boxes) >= 2:
                left_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_left_box = left_boxes[0]  # Select box with smallest box[3]
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
                right_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_right_box = right_boxes[0]  # Select box with smallest box[3]
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
                if width * height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_right_box[0])
                    detected_obj.ymin = int(selected_right_box[1])
                    detected_obj.xmax = int(selected_right_box[2])
                    detected_obj.ymax = int(selected_right_box[3])
                    detected_obj.ns = results[0].names[int(selected_right_box[5])]
                    traffic_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_right_box)
            
            self.obj_pub_combined.publish(traffic_msg)

            for box in selected_boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                label = results[0].names[int(cls)]
                cv2.rectangle(combined_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)  # Draw traffic boxes in red
                cv2.putText(combined_img, f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # Process delivery boxes
            delivery_list = ['delivery_a1', 'delivery_a2', 'delivery_a3', 'delivery_b1', 'delivery_b2', 'delivery_b3']
            
            delivery_msg = Delivery()
            delivery_msg.header.stamp = img_time

            for box in boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                print(cls)
                label = results[0].names[int(cls)]
                if conf > 0.3 and label in delivery_list:
                    cv2.rectangle(combined_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (255, 0, 0), 2)  # Draw delivery boxes in blue
                    cv2.putText(combined_img, f"{label} {conf:.2f}", (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                    detected_delivery = obj_info()
                    detected_delivery.xmin = int(xmin)
                    detected_delivery.ymin = int(ymin)
                    detected_delivery.xmax = int(xmax)
                    detected_delivery.ymax = int(ymax)
                    detected_delivery.ns = label
                    delivery_msg.obj.append(detected_delivery)

            self.delivery_pub_combined.publish(delivery_msg)
            
            # imgmsg = self.bridge.cv2_to_imgmsg(combined_img, 'bgr8')  # Convert combined image to ROS image message
            # imgmsg.header.stamp = img_time  # Set timestamp for image message
            # self.obj_pub_combined_img.publish(imgmsg)  # Publish combined image
            
            self.visualize_results(combined_img, "combined")    

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

    def visualize_results(self, img, mode):
        try:
            # Display the image with a title
            title = f"Detected Objects ({mode})"
            cv2.imshow(title, img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Visualization error: {e}")

    def stop(self):
        self.stop_event.set()
        self.image_processing_thread.join()
        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = DualWebcamObjectDetectionROS()
    rospy.spin()
    detector.stop()
