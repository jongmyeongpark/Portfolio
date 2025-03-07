#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import cv2
import numpy as np
import torch
import time
from macaron_06.msg import Traffic, obj_info
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
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
        self.obj_pub_combined = rospy.Publisher('/traffic_obj_combined', Traffic, queue_size=1)

        # YOLOv8 모델 초기화
        self.WEIGHT_PATH = "/home/macaron/pt/2023_best_traffic.pt"
        self.model = YOLO(self.WEIGHT_PATH)


        # 모델을 GPU로 강제 전환 (필요한 경우 CPU로 변경 가능)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        # ROS 이미지 토픽 구독자 설정
        self.img_sub_webcam1 = rospy.Subscriber(webcam1_topic, Image, self.img_callback_webcam1, queue_size=1)
        self.img_sub_webcam2 = rospy.Subscriber(webcam2_topic, Image, self.img_callback_webcam2, queue_size=1)
        self.sub_sign_state = rospy.Subscriber('/sign', String, self.state_callback, queue_size=1)

        self.stop_event = threading.Event()

        # Start image processing thread
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def img_callback_webcam1(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock1:
                self.latest_img_webcam1 = img
                self.latest_img_time_webcam1 = img_msg.header.stamp
            # rospy.loginfo("Webcam1 image received.")
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam1 error: {e}")

    def img_callback_webcam2(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock2:
                self.latest_img_webcam2 = img
                self.latest_img_time_webcam2 = img_msg.header.stamp
            # rospy.loginfo("Webcam2 image received.")
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
            start_time = time.time()
            ns_list = ['green', 'yellow', 'red', 'all_green', 'left']
            img = cv2.resize(img, (1280, 480))  # Resize to fit the combined image width
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            input_img = input_img.astype(np.float32) / 255.0
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)
            results = self.model(input_tensor, verbose=False)
            boxes = results[0].boxes.cpu().numpy().data

            obj_msg = Traffic()
            obj_msg.header.stamp = img_time
            
            selected_boxes = [] 
            
            left_boxes = [box for box in boxes if box[2] < 640 and box[4] > 0.3 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            right_boxes = [box for box in boxes if box[2] >= 640 and box[4] > 0.3 and results[0].names[int(box[5])] in ns_list and int(box[3]) <= 240]
            # print(f"left_boxes: {left_boxes}")
            # print(f"right_boxes: {right_boxes}")

            if len(left_boxes) >= 2:
                left_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_left_box = left_boxes[0]  # Select the box with the smallest box[3]
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_left_box[0])
                detected_obj.ymin = int(selected_left_box[1])
                detected_obj.xmax = int(selected_left_box[2])
                detected_obj.ymax = int(selected_left_box[3])
                detected_obj.ns = results[0].names[int(selected_left_box[5])]
                obj_msg.obj.append(detected_obj)
                selected_boxes.append(selected_left_box)
            
            elif len(left_boxes) == 1:
                selected_left_box = left_boxes[0]
                width =  int(selected_left_box[2]) - int(selected_left_box[0])
                height = int(selected_left_box[3]) - int(selected_left_box[1])
                # 너비 한계 관련 수치 정하기 : ????
                print(f"넓이:{width * height}")
                if width*height > 0:
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_left_box[0])
                    detected_obj.ymin = int(selected_left_box[1])
                    detected_obj.xmax = int(selected_left_box[2])
                    detected_obj.ymax = int(selected_left_box[3])
                    detected_obj.ns = results[0].names[int(selected_left_box[5])]
                    obj_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_left_box)

            else:
                pass

            if len(right_boxes) >= 2:
                right_boxes.sort(key=lambda x: x[3])  # Sort by box[3]
                selected_right_box = right_boxes[0]  # Select the box with the smallest box[3]
                detected_obj = obj_info()
                detected_obj.xmin = int(selected_right_box[0])
                detected_obj.ymin = int(selected_right_box[1])
                detected_obj.xmax = int(selected_right_box[2])
                detected_obj.ymax = int(selected_right_box[3])
                detected_obj.ns = results[0].names[int(selected_right_box[5])]
                obj_msg.obj.append(detected_obj)
                selected_boxes.append(selected_right_box)
            
            elif len(right_boxes) == 1:
                selected_right_box = right_boxes[0]
                width =  int(selected_right_box[2]) - int(selected_right_box[0])
                height = int(selected_right_box[3]) - int(selected_right_box[1])
                print(f"넓이:{width * height}")
                # 너비 한계 관련 수치 정하기: ???
                if width*height > 0:  
                    detected_obj = obj_info()
                    detected_obj.xmin = int(selected_right_box[0])
                    detected_obj.ymin = int(selected_right_box[1])
                    detected_obj.xmax = int(selected_right_box[2])
                    detected_obj.ymax = int(selected_right_box[3])
                    detected_obj.ns = results[0].names[int(selected_right_box[5])]
                    obj_msg.obj.append(detected_obj)
                    selected_boxes.append(selected_right_box)
            else:
                pass

            self.obj_pub_combined.publish(obj_msg)
            self.visualize_results(img, selected_boxes, "combined")
            end = time.time()
            print('elapsed time: ', end - start_time)
        except Exception as e:
            self.visualize_results_stop_state(img, "combined")
            rospy.logerr(f"YOLO detection error: {e}")

    def visualize_results(self, img, boxes, mode):
        try:
            ns_list = ['green', 'yellow', 'red', 'all_green', 'left']
            for box in boxes:
                xmin, ymin, xmax, ymax, conf, cls = box
                if int(cls) < 5:
                    # Draw bounding box
                    cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 0), 2)
                    # Draw class name above the bounding box
                    cv2.putText(img, ns_list[int(cls)], (int(xmin), int(ymin) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    # Draw confidence score above the class name
                    cv2.putText(img, f"{conf:.2f}", (int(xmin), int(ymin) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                                        # Draw stop state
                    text = f"State: {self.detect_sign}"
                    cv2.putText(img, text, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    #cv2.putText(img, str(self.detect_sign), (int(xmin), int(ymin)-45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
            
            
            cv2.imshow(mode, img)
            if cv2.waitKey(1) == 27:
                print("traffic_objected.py webcam shut down")

        except Exception as e:
            rospy.logerr(f"Error in visualize_results: {e}")
    
    def visualize_results_stop_state(self, img, mode):
        try:
            text = f"Stop State: {self.detect_sign}"
            cv2.putText(img, text, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(mode, img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error in visualize_results: {e}")

    def process_images(self):
        while not self.stop_event.is_set():
            try:
                latest_img_webcam1 = None
                latest_img_webcam2 = None
                latest_img_time_webcam1 = None
                latest_img_time_webcam2 = None

                with self.lock1:
                    if self.latest_img_webcam1 is not None:
                        latest_img_webcam1 = self.latest_img_webcam1.copy()
                        latest_img_time_webcam1 = self.latest_img_time_webcam1

                with self.lock2:
                    if self.latest_img_webcam2 is not None:
                        latest_img_webcam2 = self.latest_img_webcam2.copy()
                        latest_img_time_webcam2 = self.latest_img_time_webcam2

                if latest_img_webcam1 is not None and latest_img_webcam2 is not None:
                    combined_img = self.combine_images(latest_img_webcam1, latest_img_webcam2)
                    combined_img_time = latest_img_time_webcam1 if latest_img_time_webcam1 > latest_img_time_webcam2 else latest_img_time_webcam2
                    self.yolo_detection_combined(combined_img, img_time=combined_img_time)
            except Exception as e:
                rospy.logerr(f"Error in process_images: {e}")

    def cleanup(self):
        self.stop_event.set()
        self.image_processing_thread.join()
        rospy.signal_shutdown("Shutting down")

def main():
    try:
        ob = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt detected. Shutting down...")
        ob.cleanup()

if __name__ == "__main__":
    main()
if __name__ == "__main__":  
    main()
