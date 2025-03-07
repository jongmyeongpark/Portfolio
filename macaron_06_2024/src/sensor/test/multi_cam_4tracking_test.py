#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy  
import cv2  
import numpy as np  
import torch  
from macaron_06.msg import Cone, obj_info  
from sensor_msgs.msg import Image  
from std_msgs.msg import Bool, String  
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

        self.detect_sign = None

        self.lock1 = threading.Lock()  
        self.lock2 = threading.Lock()  
        self.obj_pub_combined = rospy.Publisher('cone_obj_combined', Cone, queue_size=1)  
        self.obj_pub_combined_img  = rospy.Publisher('/webcam_combined/image_raw', Image, queue_size=1)  

        self.WEIGHT_PATH = "/home/takrop/catkin_ws/src/macaron_06/src/sensor/pre/tracking/traffic_cone_yolov8s.pt"
        self.model = YOLO(self.WEIGHT_PATH)  

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'  
        self.model.to(self.device)  

        self.img_sub_webcam1 = Subscriber(webcam1_topic, Image)  
        self.img_sub_webcam2 = Subscriber(webcam2_topic, Image)  

        self.stop_event = threading.Event()  

        self.image_processing_thread = threading.Thread(target=self.process_images)  
        self.image_processing_thread.start()  

        self.sync = ApproximateTimeSynchronizer([self.img_sub_webcam1 , self.img_sub_webcam2], queue_size=10, slop=0.1)  
        self.sync.registerCallback(self.callback)  

    def callback(self, img_msg1, img_msg2):
        self.img_callback_webcam1(img_msg1)  
        self.img_callback_webcam2(img_msg2)  
    
    def img_callback_webcam1(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  
            with self.lock1:
                self.latest_img_webcam1 = img  
                self.latest_img_time_webcam1 = img_msg.header.stamp  
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam1 error: {e}")  

    def img_callback_webcam2(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  
            with self.lock2:
                self.latest_img_webcam2 = img  
                self.latest_img_time_webcam2 = img_msg.header.stamp  
        except CvBridgeError as e:
            rospy.logerr(f"img_callback_webcam2 error: {e}")  

    def combine_images(self, img1, img2):
        combined_img = np.hstack((img1, img2))  
        return combined_img  

    def yolo_detection_combined(self, img, img_time):
        try:
            ns_list = ['blue_cone', 'yellow_cone']  
            input_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  
            input_img = input_img.astype(np.float32) / 255.0  
            input_tensor = torch.tensor(input_img).permute(2, 0, 1).unsqueeze(0).to(self.device)  
            results = self.model(input_tensor, verbose=False)  
            boxes = results[0].boxes.cpu().numpy().data  

            obj_msg = Cone()  
            obj_msg.header.stamp = rospy.Time.now()
            for box in boxes:
                label = results[0].names[int(box[5])]  
                if box[4] > 0.5 and label in ns_list:  
                    detected_obj = obj_info()  
                    detected_obj.xmin = int(box[0])  
                    detected_obj.ymin = int(box[1])  
                    detected_obj.xmax = int(box[2])  
                    detected_obj.ymax = int(box[3])  
                    detected_obj.ns = label  
                    obj_msg.obj.append(detected_obj)  
            self.obj_pub_combined.publish(obj_msg)  

            imgmsg = self.bridge.cv2_to_imgmsg(img, 'bgr8')  
            imgmsg.header.stamp = rospy.Time.now()
            self.obj_pub_combined_img.publish(imgmsg)  
        except Exception as e:
            rospy.logerr(f"YOLO detection error: {e}")  

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
                    combined_img_time = rospy.Time.now()
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

if __name__ == "__main__":
    main()  