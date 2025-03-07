#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from pyproj import Proj, Transformer, CRS
import numpy as np
import rospy
from math import pi, atan2
from tf.transformations import euler_from_quaternion


heading_offset = 0   #오른쪽 +, 왼쪽 -
DIST = 0.1           # 헤딩 기록 구간
RECORD_NUMBER = 2   # 헤딩 기록 개수
STRAIGHT_ANGLE = 5  # 직진 판정 각도
VAL_WEIGHT = 2.3

MAX_ERP=30
ERP_OFFSET=0.5

#######

class gps_imu_fusion:
    def __init__(self):
        # Projection definition
        # UTM-K
        # self.proj_UTMK = Proj(init='epsg:5179')
        self.proj_UTMK = CRS('epsg:5179') #Proj(init='epsg:5179')
        
        # WGS1984
        # self.proj_WGS84 = Proj(init='epsg:4326')
        self.proj_WGS84 = CRS('epsg:4326') #Proj(init='epsg:4326')
        self.transformer = Transformer.from_crs(self.proj_WGS84, self.proj_UTMK)
        self.b = np.zeros((RECORD_NUMBER, 4))
        #2행 4열 행렬 모든 값 0
        self.c = np.zeros((10,1))
        #10행 1열 행렬 모든 값 0
        #####################0401 수정######################################
        self.sub_erp_gear = 1

    ###################################################################

    def tf_to_tm(self,lon,lat):
        # x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
        
        y, x = self.transformer.transform(lat, lon)
        # print(x, y)
        return x,y
    #pyproj library 의 transform 기능 참조

    def tf_heading_to_rad(self, head):
        heading = 5*pi/2 - np.deg2rad(float(head / 100000))
        if heading > 2*pi:
            heading -= 2*pi
        
        #####################0401 수정######################################
        #후진상태일때 gps heading값 180도 돌림#
        if(self.sub_erp_gear == 2) :
            heading += pi
            if heading > 2 * pi:
                heading -= (2 * pi)
            elif heading <= 0:
                heading += (2 * pi)
        ####################################################################

        return heading

    def q_to_yaw(self, imu):
        #q means Quaternion
        # orientation_list = [imu.x, imu.y, imu.z, imu.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # print(yaw)

        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.sub_imu_heading = yaw

        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)



        return yaw

    def heading_correction(self, x, y, imu_heading):
        global heading_offset

        if self.b[0][0] == 0 and self.b[0][1]==0:
                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = imu_heading

        else:
            distance = np.hypot(self.b[0][0] - x, self.b[0][1] - y)
            #두 점 사이의 직선거리 numpy hypot 참조. 유클리드 거리
            #두 데이터간 각 특성의 차이

            if distance >= DIST:
                
                for i in range(RECORD_NUMBER - 1, -1, -1) :
                    self.b[i][0] = self.b[i-1][0]
                    self.b[i][1] = self.b[i-1][1]
                    self.b[i][2] = self.b[i-1][2]
                    self.b[i][3] = self.b[i-1][3]
                
                # 모라이는 gps헤딩이 안들어와 변위 방향으로 gps헤딩 뽑아냄(0~2*pi)
                gps_heading = atan2(y-self.b[1][1], x-self.b[1][0])
                if gps_heading < 0:
                    gps_heading += 2*pi

                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = gps_heading
                
                # print("x :",x)
                # print("y :",y)
                # print("imu :",imu_heading)
                # print("gps :",gps_heading)
                # print("-------------------------")

                if self.b[RECORD_NUMBER - 1][0] != 0 or self.b[RECORD_NUMBER - 1][1] != 0:

                    max_heading = np.max(self.b, axis=0)
                    min_heading = np.min(self.b, axis=0)

                    # print(max_heading, min_heading)


                    if (max_heading[3] - min_heading[3] < STRAIGHT_ANGLE*pi/180) and (max_heading[2] - min_heading[2] < STRAIGHT_ANGLE*pi/180) : 
                        # avg_heading = np.mean(self.b, axis=0)
                        # heading_offset = avg_heading[3] - avg_heading[2]
                        
                        var_heading = np.var(self.b, axis=0)
                        avg_heading = np.mean(self.b, axis=0)

                        # x = Symbol('x')
                        # f = exp(-(x-avg_heading[3])**2/(2*var_heading[3]**2))/(var_heading[3]*sqrt(2*pi))

                        if (avg_heading[2] < avg_heading[3] - VAL_WEIGHT*var_heading[3]) :
                            heading_offset = (avg_heading[3] - VAL_WEIGHT*var_heading[3]) - avg_heading[2]  

                        elif (avg_heading[2] > avg_heading[3] + VAL_WEIGHT*var_heading[3]) :
                            heading_offset = (avg_heading[3] + VAL_WEIGHT*var_heading[3]) - avg_heading[2] 

                        else :
                            heading_offset = 0

                        #     print(var_heading[3])



                        # Integral(f, (x, 3, 7)).doit().evalf()
                        # heading_offset = 0

                        # self.b = np.zeros((RECORD_NUMBER, 4))



        # print(self.b)
        # print(heading_offset)
        # heading_offset = 0        
        return heading_offset



    def get_heading(self, x, y, imu_heading, i):
        global heading_offset
        #imu_heading = self.q_to_yaw(imu_orientation) 
        heading = imu_heading

        # heading = heading + (heading_offset * pi / 180)
        if heading > 2 * pi:
            heading = heading - (2 * pi)
        elif heading <= 0:
            heading = heading + (2 *pi)


        off_temp = heading_offset
        heading_offset = self.heading_correction(x, y, heading)


        if self.c[0][0] == 0 and abs(heading_offset) < 30*pi/180:
            self.c[0][0] = heading_offset

        else:
            
            if abs(heading_offset) < 30*pi/180:
                for i in range(10 - 1, -1, -1) :
                    self.c[i][0] = self.c[i-1][0]

                self.c[0][0] = heading_offset

                if self.c[9][0] != 0:
                    avg_heading_offset = np.mean(self.c, axis=0)

                    heading_offset = avg_heading_offset[0]


        #print(self.c)

        if abs(heading_offset) > 30*pi/180:
            heading_offset = off_temp

        heading = heading + heading_offset
        # heading = heading
        # heading += np.deg2rad(30)
        # # print(heading)gps_imu_fusion
        # print("fusion_heading:",heading)
        # print("-------------------------")
        return heading