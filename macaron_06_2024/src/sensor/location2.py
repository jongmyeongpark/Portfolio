#!/usr/bin/env python
# -- coding: utf-8 --
# import rospy
from pyproj import Proj, transform
from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
from math import pi

from macaron_6.msg import erp_read

heading_offset = 0   #오른쪽 +, 왼쪽 -
DIST = 0.3           # 헤딩 기록 구간
RECORD_NUMBER = 4   # 헤딩 기록 개수
STRAIGHT_ANGLE = 5  # 직진 판정 각도
VAL_WEIGHT = 2.3

MAX_ERP=30
ERP_OFFSET=0.5

#######

class gps_imu_fusion:
    def __init__(self):
        #Projection definition
        #UTM-K
        self.proj_UTMK = Proj(init='epsg:5179')
        #WGS1984
        self.proj_WGS84 = Proj(init='epsg:4326')

        self.b = np.zeros((RECORD_NUMBER, 4))
        #2행 4열 행렬 모든 값 0
        self.c = np.zeros((10,1))
        #10행 1열 행렬 모든 값 0
        #####################0401 수정######################################
        self.sub_erp_gear = 1

        self.erp_sub= rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        ###################################################################
        self.heading_offset_array = []
        self.heading_offset_mean = 0

        process_variance = 1e-5  # 프로세스 노이즈 공분산
        measurement_variance = 0.0001  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
        # measurement_variance = 0.0001
        estimation_error = 1.0  # 초기 추정 오차
        initial_value = 0 # 초기 값게

        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    #####################0401 수정######################################
    def erp_callback(self, erp) :
        self.sub_erp_gear = erp.read_gear
    ###################################################################

    def tf_to_tm(self,lon,lat):
        x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
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
        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)

        # print(yaw)

        return yaw

    def q_to_rpy(self, imu_q):
        orientation_list = [imu_q.x, imu_q.y, imu_q.z, imu_q.w]
        _,_,yaw = euler_from_quaternion(orientation_list)
        if yaw < 0:
            yaw = pi + (pi + yaw)

        # print(yaw)

        return yaw
    
    def update_KF(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value    
    
    def heading_correction(self, x, y, imu_heading, gps_heading):
        global heading_offset

        if self.b[0][0] == 0 and self.b[0][1]==0:
                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = gps_heading

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

                self.b[0][0] = x
                self.b[0][1] = y
                self.b[0][2] = imu_heading
                self.b[0][3] = gps_heading

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
                        
                        #####################################################################################
                        # self.heading_offset_array.append(heading_offset)
                        # print("___________________________________________",self.heading_offset_array)

                        # if self.heading_offset_array != []:
                        #     self.heading_offset_mean = np.mean(self.heading_offset_array)
                        # else:
                        #     self.heading_offset_mean = heading_offset

                        # heading_offset = self.heading_offset_mean
                        #####################################################################################

                        #     print(var_heading[3])



                        # Integral(f, (x, 3, 7)).doit().evalf()
                        # heading_offset = 0

                        # self.b = np.zeros((RECORD_NUMBER, 4))
                    else:
                        self.heading_offset_array = []
        # print(self.b)
        #print("!!!!!!!!!!!!!!!!!!!!!!heading_offset = ",heading_offset)
        # heading_offset = 0        
        return heading_offset

    def imu_filtering(self, imu_heading):
        heading=imu_heading

        if abs(self.b[0][2]-self.b[1][2]) > MAX_ERP+ERP_OFFSET:
            heading=self.q_to_rpy(self.b[1][2])   #imu_heading이 erp 최대각보다 커지면 heading값을 이전 값으로 가지도록 함.

        return heading


    def get_heading(self, x, y, imu_orientation, gps, i):
        global heading_offset
        gps_heading = self.tf_heading_to_rad(gps)
        imu_heading = self.q_to_rpy(imu_orientation) #imu heading update

        #imu_filtering
        heading = self.imu_filtering(imu_heading)

        heading = imu_heading
        # print(heading), 'imu heading'

        # heading = heading + (heading_offset * pi / 180)
        if heading > 2 * pi:
            heading = heading - (2 * pi)
        elif heading <= 0:
            heading = heading + (2 *pi)


        off_temp = heading_offset
        heading_offset = self.heading_correction(x, y, heading, gps_heading)
        
        if heading_offset > pi:
            heading_offset -= 2*pi
        elif heading_offset < -pi:
            heading_offset += 2*pi


        if self.c[0][0] == 0 and abs(heading_offset) < 2 * pi:
            self.c[0][0] = heading_offset

        else:
            
            if abs(heading_offset) < 2 * pi:
                for i in range(10 - 1, -1, -1) :
                    self.c[i][0] = self.c[i-1][0]

                self.c[0][0] = heading_offset

                if self.c[9][0] != 0:
                    avg_heading_offset = np.mean(self.c, axis=0)

                    heading_offset = avg_heading_offset[0]


        # print(self.c)

        # print("gps:",gps_heading)
        # print("imu:",imu_heading)
        
        # print("offset: ",heading_offset)

        if abs(heading_offset) > 2 * pi:
            heading_offset = off_temp

        heading = heading + heading_offset

        heading = self.update_KF(heading)

        if (abs(gps_heading - heading) > 0.1) and (abs(gps_heading - heading) < pi/2):
            heading = gps_heading
            print("_______________________________________________________-GPS_heading")

        print("raw_heading"," GPS : ",gps_heading,"IMU : ",imu_heading,"heading : ",heading)
        # heading = heading
        # heading += np.deg2rad(30)
        # # print(heading)gps_imu_fusion
        return heading

######################

# #!/usr/bin/env python
# # -- coding: utf-8 --
# # import rospy
# from pyproj import Proj, transform
# # from tf.transformations import euler_from_quaternion
# # from geometry_msgs.msg import Quaternion
# import numpy as np
# from math import pi

# heading_offset = 0   #오른쪽 +, 왼쪽 -
# DIST = 0.3           # 헤딩 기록 구간
# RECORD_NUMBER = 2   # 헤딩 기록 개수
# STRAIGHT_ANGLE = 5  # 직진 판정 각도
# VAL_WEIGHT = 3

# class gps_imu_fusion:
#     def __init__(self):
#         #Projection definition
#         #UTM-K
#         self.proj_UTMK = Proj(init='epsg:5179')
#         #WGS1984
#         self.proj_WGS84 = Proj(init='epsg:4326')

#         self.b = np.zeros((RECORD_NUMBER, 4))
#         #2행 4열 행렬 모든 값 0
#         self.c = np.zeros((10,1))
#         #10행 1열 행렬 모든 값 0

#     def tf_to_tm(self,lon,lat):
#         x,y=transform(self.proj_WGS84,self.proj_UTMK,lon,lat)
#         return x,y
#     #pyproj library 의 transform 기능 참조

#     def tf_heading_to_rad(self, head):
#         heading = 5*pi/2 - np.deg2rad(float(head / 100000))
#         if heading > 2*pi:
#             heading -= 2*pi
#         return heading

#     def q_to_yaw(self, imu):
#         #q means Quaternion
#         # orientation_list = [imu.x, imu.y, imu.z, imu.w]
#         # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#         # print(yaw) 
#         yaw = (-1) * imu.x * pi / 180

#         if yaw < 0:
#             yaw = pi + (pi + yaw)

#         # print(yaw)

#         return yaw

#     def heading_correction(self, x, y, imu_heading, gps_heading):
#         global heading_offset

#         if self.b[0][0] == 0 and self.b[0][1]==0:
#                 self.b[0][0] = x
#                 self.b[0][1] = y
#                 self.b[0][2] = imu_heading
#                 self.b[0][3] = gps_heading

#         else:
#             distance = np.hypot(self.b[0][0] - x, self.b[0][1] - y)
#             #두 점 사이의 직선거리 numpy hypot 참조. 유클리드 거리
#             #두 데이터간 각 특성의 차이

#             if distance >= DIST:
                
#                 for i in range(RECORD_NUMBER - 1, -1, -1) :
#                     self.b[i][0] = self.b[i-1][0]
#                     self.b[i][1] = self.b[i-1][1]
#                     self.b[i][2] = self.b[i-1][2]
#                     self.b[i][3] = self.b[i-1][3]

#                 self.b[0][0] = x
#                 self.b[0][1] = y
#                 self.b[0][2] = imu_heading
#                 self.b[0][3] = gps_heading

#                 if self.b[RECORD_NUMBER - 1][0] != 0 or self.b[RECORD_NUMBER - 1][1] != 0:

#                     max_heading = np.max(self.b, axis=0)
#                     min_heading = np.min(self.b, axis=0)

#                     print(max_heading, min_heading)


#                     if (max_heading[3] - min_heading[3] < STRAIGHT_ANGLE*pi/180) and (max_heading[2] - min_heading[2] < STRAIGHT_ANGLE*pi/180) :

#                         # avg_heading = np.mean(self.b, axis=0)
#                         # heading_offset = avg_heading[3] - avg_heading[2]
                        
#                         var_heading = np.var(self.b, axis=0)
#                         avg_heading = np.mean(self.b, axis=0)

#                         # x = Symbol('x')
#                         # f = exp(-(x-avg_heading[3])**2/(2*var_heading[3]**2))/(var_heading[3]*sqrt(2*pi))

#                         if (avg_heading[2] < avg_heading[3] - VAL_WEIGHT*var_heading[3]) :
#                             heading_offset = (avg_heading[3] - VAL_WEIGHT*var_heading[3]) - avg_heading[2]  

#                         elif (avg_heading[2] > avg_heading[3] + VAL_WEIGHT*var_heading[3]) :
#                             heading_offset = (avg_heading[3] + VAL_WEIGHT*var_heading[3]) - avg_heading[2] 

#                         else :
#                             heading_offset = 0

#                         #     print(var_heading[3])



#                         # Integral(f, (x, 3, 7)).doit().evalf()
#                         # heading_offset = 0

#                         # self.b = np.zeros((RECORD_NUMBER, 4))
                        


#         # print(self.b)
#         print(heading_offset)
#         # heading_offset = 0        
#         return heading_offset

#     def get_heading(self, x, y, imu_orientation, gps, i):
#         global heading_offset
#         gps_heading = self.tf_heading_to_rad(gps)
#         imu_heading = self.q_to_yaw(imu_orientation) #imu heading update


#         heading = imu_heading
#         print(heading), 'imu heading'    

#         # heading = heading + (heading_offset * pi / 180)
#         if heading > 2 * pi:
#             heading = heading - (2 * pi)
#         elif heading <= 0:
#             heading = heading + (2 *pi)


#         off_temp = heading_offset
#         heading_offset = self.heading_correction(x, y, heading, gps_heading)


#         if self.c[0][0] == 0 and abs(heading_offset) < 30*pi/180:
#             self.c[0][0] = heading_offset

#         else:
            
#             if abs(heading_offset) < 30*pi/180:
#                 for i in range(10 - 1, -1, -1) :
#                     self.c[i][0] = self.c[i-1][0]

#                 self.c[0][0] = heading_offset

#                 if self.c[9][0] != 0:
#                     avg_heading_offset = np.mean(self.c, axis=0)

#                     heading_offset = avg_heading_offset[0]


#         print(self.c)




#         if abs(heading_offset) > 30*pi/180:
#             heading_offset = off_temp

#         heading = heading + heading_offset
#         # heading = heading
                        
#         # # print(heading)gps_imu_fusion
#         return heading
