#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
import os, sys
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/sensor")
from macaron_5.msg import erp_read, obj_info
from sub_erp_state import sub_erp_state

point_1 = [935654.06, 1916124.191]
point_2 = [935656.7918, 1916157.7790]
point_3 = [935656.6329, 1916166.8162]
point_4 = [935656.025, 1916200.538]



def distance(mission_pose, current_pose):
    x_gap = mission_pose[0] - current_pose[0]
    y_gap = mission_pose[1] - current_pose[1]
    dis = np.hypot(x_gap, y_gap)
    return dis


class pub_delivery:
    def __init__(self):
        self.del_pub = rospy.Publisher('/class', obj_info, queue_size=1)
        self.sign = obj_info()
    def pub_sign(self, Type):
        if(Type == 1):
            self.sign.ns = "delivery1"
            self.sign.xmin = 0
            self.sign.xmax = 0
            self.sign.ymin = 0
            self.sign.ymax = 0
            self.del_pub.publish(self.sign)        
        elif(Type == 2):
            self.sign.ns = "delivery1"
            self.sign.xmin = 120
            self.sign.xmax = 0
            self.sign.ymin = 0
            self.sign.ymax = 0
            self.del_pub.publish(self.sign)
        elif(Type == 3):
            self.sign.ns = "delivery3"
            self.sign.xmin = 0
            self.sign.xmax = 0
            self.sign.ymin = 0
            self.sign.ymax = 0
            self.del_pub.publish(self.sign)
        elif(Type == 4):
            self.sign.ns = "delivery3"
            self.sign.xmin = 120
            self.sign.xmax = 0
            self.sign.ymin = 0
            self.sign.ymax = 0
            self.del_pub.publish(self.sign)
        
def main():
    pub = pub_delivery()
    erp = sub_erp_state()
    print("배달 표지판 정보 pub start")

    while not rospy.is_shutdown():
 
        if(distance(erp.pose,point_1)<=1):  #어느정도 가다가 A표지판 신호 pub 하기
            print("A_1 표지판 발견")
            pub.pub_sign(1)
        elif(distance(erp.pose,point_2)<=2):
            print("A_1 표지판으로")
            pub.pub_sign(2)
        elif(distance(erp.pose,point_3)<=1):
            print("B_2 표지판 발견")
            pub.pub_sign(3)
        elif(distance(erp.pose,point_4)<=2):
            print("B_2 표지판으로")
            pub.pub_sign(4)
        else:
            pass

        rospy.sleep(0.5)
       
if __name__ == '__main__':
    rospy.init_node('pub_del_sign', anonymous=True)
    main()



    