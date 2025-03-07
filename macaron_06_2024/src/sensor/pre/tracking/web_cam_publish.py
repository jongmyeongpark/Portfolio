#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DualWebcamROS():
    def __init__(self):
        self.bridge = CvBridge()
        self.cap1 = cv2.VideoCapture(0)  # 첫 번째 웹캠 (인덱스 4)
        self.cap2 = cv2.VideoCapture(4)  # 두 번째 웹캠 (인덱스 6)
        self.pub1 = rospy.Publisher('/webcam1/image_raw', Image, queue_size=1)
        self.pub2 = rospy.Publisher('/webcam2/image_raw', Image, queue_size=1)
        
        rospy.init_node('dual_webcam_ros')
        rospy.on_shutdown(self.cleanup)

    def publish_images(self):
        rate = rospy.Rate(30)  # 초당 30번의 메시지 전송
        
        while not rospy.is_shutdown():
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()
            
            if ret1:
                try:
                    self.pub1.publish(self.bridge.cv2_to_imgmsg(frame1, 'bgr8'))
                    cv2.imshow("Webcam 1", frame1)
                    print("webcam1")
                except CvBridgeError as e:
                    rospy.logerr(e)
            
            if ret2:
                try:
                    self.pub2.publish(self.bridge.cv2_to_imgmsg(frame2, 'bgr8'))
                    cv2.imshow("Webcam 2", frame2)
                    print("webcam2")
                except CvBridgeError as e:
                    rospy.logerr(e)
            
            if ret1 and ret2:
                print("Dual Webcam Opened!")

            if cv2.waitKey(1) == 27:
                print("web_cam_publish.py webcam shut down")
                break
            rate.sleep()
        cv2.destroyAllWindows()

    def cleanup(self):
        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()

    def run(self):
        rospy.loginfo("Dual Webcam ROS Node Started")
        self.publish_images()

if __name__ == '__main__':
    try:
        dual_webcam_ros = DualWebcamROS()
        dual_webcam_ros.run()
    except rospy.ROSInterruptException:
        pass
