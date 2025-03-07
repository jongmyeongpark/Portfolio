#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud, Imu, MagneticField
from geometry_msgs.msg import Point, Quaternion

from tf.transformations import euler_from_quaternion


class SensorDataHub:
    def __init__(self):
        #구독자 선언
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_mag = rospy.Subscriber('/imu/mag', MagneticField, self.mag_callback, queue_size=1)
        
        self.pub_imu_yaw = rospy.Publisher('/imu_yaw', Float32, queue_size=1)
        self.x = []
        self.y = []

        self.imu_yaw = 0.0
        self.mag_yaw = 0.0

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC
        self.erp_INPUT_TIME = time.time()

    def imu_callback(self, imu):
        # orientation = (imu.orientation.x-0.00264, imu.orientation.y-0.00264, imu.orientation.z, imu.orientation.w)
        orientation = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        self.x.append(imu.orientation.x)
        self.y.append(imu.orientation.y)
        _,_,yaw = euler_from_quaternion(orientation)
        # yaw = math.atan2(imu.orientation.y, imu.orientation.x)
        # yaw = -yaw
        # yaw += np.deg2rad(-8.0 + (50.0/60.0)) 

        # yaw = (2*np.pi - yaw) - 3.976
        # yaw += np.pi *0.5
        if yaw >= 2*np.pi: yaw -= 2*np.pi
        if yaw < 0: yaw += 2*np.pi
        self.imu_yaw = yaw
        # print(f'deg: {heading}')
        # print(f'rad: {yaw}')
        # print()
        self.pub_imu_yaw.publish(yaw)

    def mag_callback(self, mag):
        pi = np.pi
        yaw = np.arctan2(mag.magnetic_field.y, mag.magnetic_field.x) + pi * 0.5

        if yaw >= 2 * pi:
            yaw -= 2*pi

        if yaw < 0:
            yaw += 2*pi
        self.mag_yaw = yaw
        # print(f'deg: {heading}')
        # print(f'mag_rad: {yaw}')
        # print()
        # self.pub_imu_yaw.publish(yaw)

def main():
    #기본 설정
    rospy.init_node('data_hub', anonymous=True)
    # rate=rospy.Rate(1) #지금 코드에서는 안써도 됨
    # rate = rospy.Rate(10)
    Data = SensorDataHub() # 객체 생성
    rate = rospy.Rate(10)
    start_rate = time.time() # 시간 스템프
    i_list = []
    m_list = []
    # 센서가 모두 동작하기 전까지는 대기
    # while not Data.senser_check():
    #     print("senser not ready")
    #     rate.sleep()
    #     continue

    while not rospy.is_shutdown():

            # 각 정보 업데이트 후 발행
        print(f'imu_yaw: {Data.imu_yaw}')
        print(f'mag_yaw: {Data.mag_yaw}')
        print(f'diff: {Data.imu_yaw-Data.mag_yaw}')
        print(f'=======================================')
        print()
        i_list.append(Data.imu_yaw)
        m_list.append(Data.mag_yaw)

        rate.sleep()

    time_list = np.arange(len(i_list))
    plt.plot(time_list , i_list, "r")
    plt.plot(time_list , m_list, "b")
    plt.show()
    first_index = Data.x.index(min(Data.x))
    max_x_index = Data.x.index(max(Data.x)) 
    second_index = Data.y.index(min(Data.y))
    max_y_index = Data.y.index(max(Data.y)) 

    # plt.plot(Data.x , Data.y, "b.")
    # plt.plot([-1,1], [0,0], 'r')
    # plt.plot([0,0], [-1,1], 'r')
    # plt.plot([Data.x[first_index], Data.x[second_index]], [Data.y[first_index], Data.y[second_index]], 'r')
    # plt.show()
    # print(f'x: {(Data.x[max_x_index] - Data.x[first_index])/2}')
    # print(f'y: {(Data.y[max_y_index] - Data.y[second_index])/2}')
if __name__ == '__main__':
    main()