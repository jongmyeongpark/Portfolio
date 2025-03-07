import numpy as np
from pyquaternion import Quaternion
import rospy
import numpy as np
from scipy import stats

import time
import matplotlib.pyplot as plt

#!/usr/bin/env python
# -- coding: utf-8 --

# basic package
from datetime import date
import rospy
import time
from math import pi
import math
import numpy as np

# message file
from macaron_6.msg import erp_read
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from ublox_msgs.msg import NavPVT,NavVELNED
from math import atan2 ,pi, sin, cos
from sensor_msgs.msg import NavSatFix



from tf.transformations import euler_from_quaternion

from pyproj import Proj, Transformer, CRS

import time
import csv

proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')


class Position:
    def __init__(self):
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_gps = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.gps_callback,queue_size=1)
        # self.sub_gps_heading = rospy.Subscriber("ublox_gps/navpvt", NavPVT, self.gps_heading_callback,queue_size=1)
        self.sub_gps_vel = rospy.Subscriber("ublox_gps/navpvt",NavPVT,self.gps_data_callback,queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.GPS_VELOCITY_COUNT = 0
        self.GPS_VELOCITY_dT = 0
        self.GPS_VELOCITY_VEL = 0 #속도
        self.GPS_VELOCITY_LAST_VEL = 0 #이전 속도
        self.GPS_VECOCITY_accel = 0
        self.GPS_VECOCITY_Heading = 0

        self.pos = [0,0]
        self.last_pos = [0,0]

        self.GPS_VELOCITY_COUNT = 0
        self.GPS_DATA_dT = 0
        self.GPS_DATA_LAST_TIME = time.time()
        self.GPS_DATA_accel = 0
        self.GPS_DATA_velocity_kmh = 0 #속도
        self.GPS_DATA_LAST_velocity_kmh = 0 #이전 속도
        self.GPS_DATA_heading_degrees = 0
        self.GPS_DATA_pos_heading_radians = 0

        self.ENC_VELOCITY_COUNT = 0
        self.ENC = 0
        self.ENC_LAST_TIME = time.time()
        self.ENC_NOW_TIME = time.time()
        self.ENC_LAST_VELOCITY_VEL = 0
        self.ENC_ACCEL = 0
        self.ENC_VELOCITY_dT = 0
        self.ENC_VELOCITY_VEL = 0

        self.IMU_VELOCITY_COUNT = 0
        self.IMU_accel_x = 0
        self.IMU_accel_y = 0
        self.yaw = 0
        self.IMU_LAST_accel_x = 0
        self.IMU_LAST_accel_y = 0
        self.IMU_INPUT_TIME = time.time()
        self.IMU_LAST_TIME = time.time()
        self.IMU_VELOCITY_VEL = 0
        self.IMU_ACCEL_X_avg = 0
        self.IMU_VELOCITY_dT = 0
        self.IMU_HEADING_CORRECTION = 0

        self.GPS_move_flag = 0
        self.location_heading = 0

    def gps_callback(self, gps_msg):
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()

    def GPS_POS(self):
        return self.pos    

    def GPS_VELOCITY(self):
        if self.GPS_VELOCITY_COUNT == 0:
            while self.pos == [0,0]:
                print("WAITING")
            self.GPS_LAST_TIME = self.GPS_INPUT_TIME
            self.last_pos = self.pos
            self.GPS_VELOCITY_COUNT = 1

        elif self.GPS_VELOCITY_COUNT == 1:
            self.GPS_VELOCITY_dT = abs(self.GPS_INPUT_TIME-self.GPS_LAST_TIME)

            if self.GPS_VELOCITY_dT > 0.1:
                ####################################################
                if self.pos == self.last_pos:
                    self.GPS_move_flag = 0
                else:  
                    self.GPS_move_flag = 1  
                ####################################################

                Distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                self.GPS_VECOCITY_Heading = np.arctan2(self.pos[1] - self.last_pos[1],self.pos[0] - self.last_pos[0])

                if self.GPS_VELOCITY_dT != 0:
                    self.GPS_VECOCITY_accel = (self.GPS_VELOCITY_LAST_VEL - self.GPS_VELOCITY_VEL) / self.GPS_VELOCITY_dT /3.6

                while self.GPS_VECOCITY_Heading > 2*pi:
                    self.GPS_VECOCITY_Heading -= 2*pi
                    # print("-----------",self.GPS_VECOCITY_Heading)

                while self.GPS_VECOCITY_Heading < 0:
                    self.GPS_VECOCITY_Heading += 2*pi
                    # print("-----------",self.GPS_VECOCITY_Heading)

                self.GPS_VELOCITY_VEL = Distance / self.GPS_VELOCITY_dT * 3.6

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.GPS_VELOCITY_LAST_VEL = self.GPS_VELOCITY_VEL

        return self.GPS_VELOCITY_VEL, self.GPS_VECOCITY_accel, self.GPS_VECOCITY_Heading, self.GPS_VELOCITY_dT


    def gps_data_callback(self,nav_msg):
        self.GPS_DATA_dT = abs(self.GPS_INPUT_TIME - self.GPS_DATA_LAST_TIME)
        velocity_ms = nav_msg.gSpeed / 1000
        self.GPS_DATA_velocity_kmh = velocity_ms * 3.6

        if self.GPS_DATA_dT != 0:
            self.GPS_DATA_accel = (self.GPS_DATA_velocity_kmh-self.GPS_DATA_LAST_velocity_kmh) / self.GPS_DATA_dT / 3.6

        if self.GPS_move_flag == 1:
            pos_heading_degrees = nav_msg.heading / 1e5
            self.GPS_DATA_pos_heading_radians = np.radians(pos_heading_degrees)

            while self.GPS_DATA_pos_heading_radians > 2*np.pi:
                self.GPS_DATA_pos_heading_radians  -= 2*np.pi
                # print("-----------",self.GPS_DATA_pos_heading_radians)

            while self.GPS_DATA_pos_heading_radians < 0:
                self.GPS_DATA_pos_heading_radians  += 2*np.pi
                # print("-----------",self.GPS_DATA_pos_heading_radians)    
            # self.GPS_DATA_pos_heading_radians = 2*pi - self.GPS_DATA_pos_heading_radians 

        accuracy_velocity = nav_msg.sAcc /1000
        accuracy_heading = nav_msg.headAcc / 1e5

        # print("-----속도 정확도-----",accuracy_velocity,"-----헤딩 정확도-----",accuracy_heading)

        self.GPS_DATA_LAST_TIME = self.GPS_INPUT_TIME 
        self.GPS_DATA_LAST_velocity_kmh = self.GPS_DATA_velocity_kmh

    def GPS_DATA_VELOCITY(self):
        return self.GPS_DATA_velocity_kmh, self.GPS_DATA_accel, self.GPS_DATA_pos_heading_radians,self.GPS_DATA_dT

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.ENC = data.read_ENC
        self.ERP_INPUT_TIME = time.time()

    def ENC_VELOCITY(self):
        self.ENC_flag = 0
        if self.ENC_VELOCITY_COUNT == 0:
            self.LAST_ENC = self.ENC
            self.ENC_VELOCITY_COUNT = 1

        elif self.ENC_VELOCITY_COUNT == 1:
            if self.LAST_ENC != self.ENC:
                self.ENC_NOW_TIME = time.time()
                self.ENC_VELOCITY_dT = abs( self.ENC_NOW_TIME -  self.ENC_LAST_TIME)

                if self.ENC_VELOCITY_dT != 0:
                    self.ENC_VELOCITY_VEL = 0.5*abs(self.ENC - self.LAST_ENC)*(2*pi/100)/2/self.ENC_VELOCITY_dT * 3.6  # 바퀴 지름 50cm
                    self.ENC_ACCEL = (self.ENC_VELOCITY_VEL - self.ENC_LAST_VELOCITY_VEL) / self.ENC_VELOCITY_dT / 3.6

                self.ENC_LAST_TIME = self.ENC_NOW_TIME
                self.LAST_ENC = self.ENC
                self.ENC_LAST_VELOCITY_VEL = self.ENC_VELOCITY_VEL
                self.ENC_flag = 1

        return self.ENC_VELOCITY_VEL,self.ENC_ACCEL,self.steer,self.ENC_VELOCITY_dT,self.ENC_flag
    
    def imu_callback(self,imu):
        self.IMU_RAW_accel_x = imu.linear_acceleration.x
        self.IMU_RAW_accel_y = imu.linear_acceleration.y
        self.IMU_RAW_accel_z = imu.linear_acceleration.z

        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        self.yaw = 2*pi - (self.yaw)
        # self.yaw  = self.yaw % (2 * np.pi)

        if self.yaw  >= 2*np.pi:
            self.yaw  -= 2*np.pi
            # print("-----------",self.yaw)
        elif self.yaw  <= 0:
            self.yaw  += 2*np.pi
        
        # self.yaw = 2*pi - self.yaw
        self.yaw = self.yaw + self.IMU_HEADING_CORRECTION

        measurement =[[self.IMU_RAW_accel_x],[self.IMU_RAW_accel_y],[self.IMU_RAW_accel_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.IMU_accel_x = -int(result[0][0] * 10**a) / 10**a
        self.IMU_accel_y = int(result[1][0] * 10**a) / 10**a
        self.IMU_accel_z = int(result[2][0] * 10**a) / 10**a    

        self.IMU_INPUT_TIME = time.time()

    def IMU_VELOCITY(self):
        if self.IMU_VELOCITY_COUNT == 0:
            self.IMU_LAST_accel_x = self.IMU_accel_x
            self.IMU_LAST_accel_y = self.IMU_accel_y
            self.IMU_LAST_TIME = time.time()
            self.IMU_VELOCITY_COUNT = 1

        elif self.IMU_VELOCITY_COUNT == 1:
            self.IMU_VELOCITY_dT = abs(self.IMU_INPUT_TIME - self.IMU_LAST_TIME)    

            if self.IMU_VELOCITY_dT > 0.1:
                self.IMU_ACCEL_X_avg = (self.IMU_accel_x + self.IMU_LAST_accel_x)/2
                self.IMU_ACCEL_Y_avg = (self.IMU_accel_y + self.IMU_LAST_accel_y)/2
                self.IMU_VELOCITY_X = (self.IMU_ACCEL_X_avg) * self.IMU_VELOCITY_dT 
                self.IMU_VELOCITY_Y = (self.IMU_ACCEL_Y_avg) * self.IMU_VELOCITY_dT 

                self.IMU_VELOCITY_VEL += self.IMU_VELOCITY_X *3.6

                if self.IMU_VELOCITY_VEL < 0:
                    self.IMU_VELOCITY_VEL = 0

                self.IMU_LAST_accel_x = self.IMU_accel_x
                self.IMU_LAST_accel_y = self.IMU_accel_y
                self.IMU_LAST_VELOCITY_VEL = self.IMU_VELOCITY_VEL
                self.IMU_LAST_TIME = self.IMU_INPUT_TIME

        return self.IMU_VELOCITY_VEL,self.IMU_ACCEL_X_avg,self.yaw,self.IMU_VELOCITY_dT,
    
    def IMU_CORRECTION(self):
        return (0 - self.IMU_ACCEL_X_avg)
    
    
class KalmanFilter_ENC:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
    #     self.measurement_variance = update_measurement_variance

        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value
    
class KalmanFilter_HEADING:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
    #     self.measurement_variance = update_measurement_variance

        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value    
    
class EKF_ENC_IMU:
    def __init__(self, dt, state_dim, meas_dim, process_noise, meas_noise):
        self.dt = dt
        self.state_dim = state_dim
        self.meas_dim = meas_dim

        # 상태 벡터 초기화 (위치, 속도, 가속도)
        self.x = np.zeros((state_dim, 1))

        # 초기 추정 공분산 행렬
        self.P = np.eye(state_dim)

        # 상태 전이 행렬
        self.F = np.array([[1, dt, 0.5*dt**2],
                           [0, 1, dt],
                           [0, 0, 1]])

        # 관측 행렬
        self.H = np.array([[0, 1, 0],
                           [0, 0, 1]])

        # 상태 잡음 공분산 행렬
        self.Q = process_noise

        # 관측 잡음 공분산 행렬
        self.R = meas_noise

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = self.P - K @ self.H @ self.P

    def run(self, z, dt):
        self.dt = dt
        self.predict()
        self.update(z)
        return self.x    

class save:
    def __init__(self):
        pass

    def save_data(self, file_path,array):
        # 자기장 데이터를 CSV 파일에 저장하는 함수
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['GPS_X', 'PREDICT_X', 'GPS_Y','PREDICT_Y','X_차이','Y_차이'])
            writer.writerows(array)

    def find_max_value(self,arr):
        if len(arr) == 0:
            raise ValueError("The array is empty")

        max_value = arr[0]  # 배열의 첫 번째 요소를 최댓값으로 설정
        for num in arr:
            if num > max_value:
                max_value = num

        return max_value    
            
def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    s = save()
    rate = rospy.Rate(10)

    GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = 0,0,0,0
    GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = 0,0,0,0
    IMU_velocity,IMU_accel,IMU_heading,IMU_dt = 0,0,0,0
    ENC_velocity,ENC_accel,ENC_LOCAL_heading,ENC_dt,ENC_flag = 0,0,0,0,0

    GPS_pos_velocity_plot,GPS_pos_accel_plot,GPS_pos_heading_plot = [],[],[]
    GPS_data_velocity_plot,GPS_data_accel_plot,GPS_data_heading_plot = [],[],[]
    IMU_velocity_plot,IMU_accel_plot,IMU_heading_plot = [],[],[]
    ENC_velocity_plot,ENC_accel_plot = [],[]

    count,count_plot = 0,[]

    ###############################################################################

    process_variance = 1e-5  # 프로세스 노이즈 공분산
    # measurement_variance = 0.000375  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
    measurement_variance = 0.0001
    estimation_error = 1.0  # 초기 추정 오차
    initial_value = 0 # 초기 값게

    kf_ENC_VELOCITY = KalmanFilter_ENC(process_variance, measurement_variance, estimation_error, initial_value)

    ################################################################################

    process_variance1 = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance1 = 0.0001
    estimation_error1 = 1.0  # 초기 추정 오차
    initial_value1 = 0 # 초기 값게

    kf_HEADING = KalmanFilter_HEADING(process_variance1, measurement_variance1, estimation_error1, initial_value1)

    ################################################################################
    dt = 0.1
    # 상태 벡터 차원 (위치, 속도, 가속도)
    state_dim = 3

    # 관측 벡터 차원 (속도, 가속도)
    meas_dim = 2

    # 상태 잡음 공분산 행렬
    process_noise = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    # 관측 잡음 공분산 행렬
    meas_noise = np.array([[1, 0],
                        [0, 5]])
    
    ekf = EKF_ENC_IMU(dt, state_dim, meas_dim, process_noise, meas_noise)
    ################################################################################
    FILTERED_ENC_VELOCITY = 0
    FILTERED_ENC_VELOCITY_plot = []

    EKF_ENC_IMU_plot = []
    EKF_ENC_IMU_VEL = 0
    LAST_EKF_ENC_IMU_VEL = 0

    IMU_Correction = 0

    LAST_ENC_velocity = 0
    ENC_IMU_velocity = 0
    ENC_IMU_velocity_plot = []

    ENC_FILTER = [0,0,0]

    GPS_POS = [0,0]
    GPS_LAST_HEADING = 0
    GPS_POS_X_plot= []
    GPS_POS_Y_plot = []

    PREDICT_INIT_POS = [0,0]
    PREDICT_POS_X_plot = []
    PREDICT_POS_Y_plot = []
    
    PREDICT_POS_X = 0
    PREDICT_POS_Y = 0
    PREDICT_MOVING_X = 0
    PREDICT_MOVING_Y = 0

    PREDICT_VELOCITY_X = 0
    PREDICT_VELOCITY_Y = 0
    LAST_PREDICT_VELOCITY_X = 0
    LAST_PREDICT_VELOCITY_Y = 0

    HEADING_EDIT = 0
    SAVE_DATA_ARRAY = []


    count,count_plot = 0,[]
    HEADING_EDIT_array = []
    HEADING_EDIT = 0

    while not rospy.is_shutdown():  

        # for i in range(2):
        GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = p.GPS_VELOCITY()
        GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = p.GPS_DATA_VELOCITY()
        IMU_velocity,IMU_accel,IMU_heading,IMU_dt = p.IMU_VELOCITY()
        ENC_velocity,ENC_accel,ENC_LOCAL_heading,ENC_dt,ENC_flag = p.ENC_VELOCITY() 
        print("----------------------------------------------------------------------",GPS_data_heading,"-------",IMU_heading)
        if GPS_data_velocity < 0.3 and IMU_accel > 0:
            IMU_Correction = p.IMU_CORRECTION()

        else:
            IMU_Correction = 0

        IMU_accel = IMU_accel + IMU_Correction

        if ENC_flag == 1 or IMU_accel > 0:
            p.IMU_VELOCITY_VEL = ENC_velocity
            ENC_IMU_velocity = ENC_velocity
        else:
            ENC_IMU_velocity = ENC_IMU_velocity + IMU_accel*IMU_dt
            # print("정지",ENC_IMU_velocity)

        if ENC_velocity == 0:
            ENC_IMU_velocity = 0

        if ENC_IMU_velocity < 0:
            ENC_IMU_velocity = 0   

        FILTERED_ENC_VELOCITY =kf_ENC_VELOCITY.update(ENC_IMU_velocity)
        FILTERED_ENC_VELOCITY_plot.append(FILTERED_ENC_VELOCITY)

    ################################################################################################
        if ENC_velocity > 20:
            ENC_velocity = 0

        # print(ENC_velocity,LAST_ENC_velocity)
        if ENC_velocity != LAST_ENC_velocity: 
            ENC_FILTER[0] = ENC_FILTER[1]
            ENC_FILTER[1] = ENC_FILTER[2]
            ENC_FILTER[2] = ENC_velocity

            # print(ENC_FILTER)

            if (ENC_FILTER[1] > ENC_FILTER[0] and ENC_FILTER[1] > ENC_FILTER[2]) or (ENC_FILTER[1] < ENC_FILTER[0] and ENC_FILTER[1] < ENC_FILTER[2]):
                ENC_FILTER[1] = (ENC_FILTER[0] + ENC_FILTER[2])/2  
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                FILTERED_ENC_velocity = ENC_FILTER[1]

            else:
                FILTERED_ENC_velocity = ENC_FILTER[1]

        LAST_ENC_velocity = ENC_velocity


        obs = [FILTERED_ENC_VELOCITY,IMU_accel]
        z = np.array(obs).reshape(-1, 1)
        estimated_state = ekf.run(z,IMU_dt)
        EKF_ENC_IMU_VEL = estimated_state[1][0]

        # print("!__________________________!",IMU_accel*IMU_dt)
        Percent = 0.8
        if IMU_accel>0:
            if EKF_ENC_IMU_VEL > LAST_EKF_ENC_IMU_VEL + IMU_accel*IMU_dt*Percent:
                EKF_ENC_IMU_VEL = LAST_EKF_ENC_IMU_VEL 

        else:
            if EKF_ENC_IMU_VEL < LAST_EKF_ENC_IMU_VEL + IMU_accel*IMU_dt*Percent:
                EKF_ENC_IMU_VEL = LAST_EKF_ENC_IMU_VEL    

        if EKF_ENC_IMU_VEL < 0:
            EKF_ENC_IMU_VEL = 0

        EKF_ENC_IMU_plot.append(EKF_ENC_IMU_VEL)
        print(round(estimated_state[1][0],2),"-------------------",round(EKF_ENC_IMU_VEL,2),"-------------------",round(GPS_data_velocity,2),"-------오차율-------",(GPS_data_velocity-EKF_ENC_IMU_VEL)/GPS_data_velocity*100,"%")
        LAST_EKF_ENC_IMU_VEL = estimated_state[1][0]

        GPS_pos_velocity_plot.append(GPS_pos_velocity)
        GPS_data_velocity_plot.append(GPS_data_velocity)
        IMU_velocity_plot.append(IMU_velocity)
        ENC_velocity_plot.append(ENC_velocity)
        ENC_IMU_velocity_plot.append(ENC_IMU_velocity)   

        GPS_pos_accel_plot.append(GPS_pos_accel)
        GPS_data_accel_plot.append(GPS_data_accel)
        IMU_accel_plot.append(IMU_accel)
        ENC_accel_plot.append(ENC_accel)

        GPS_pos_heading_plot.append(GPS_pos_heading)
        GPS_data_heading_plot.append(GPS_data_heading)
        IMU_heading_plot.append(IMU_heading)

        #####################################
        DEAD_RECKONING_START = 200
        if p.GPS_move_flag == 1 and count < DEAD_RECKONING_START:
            if GPS_data_heading > IMU_heading:
                    HEADING_EDIT = GPS_data_heading - IMU_heading
                    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",HEADING_EDIT,IMU_heading,GPS_data_heading)
            elif GPS_data_heading < IMU_heading:
                HEADING_EDIT = IMU_heading - GPS_data_heading
                print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",HEADING_EDIT,IMU_heading,GPS_data_heading)

            HEADING_EDIT_array.append(HEADING_EDIT)    
            HEADING_EDIT_MEAN = np.mean(HEADING_EDIT_array)
        #####################################

        # DEAD_RECKONING_START = 350
        if count >=DEAD_RECKONING_START:
            GPS_POS = p.GPS_POS()
            if count == DEAD_RECKONING_START:
                GPS_LAST_HEADING = GPS_data_heading
                PREDICT_INIT_POS = GPS_POS
                POINT_PREDICT_INIT_POS = PREDICT_INIT_POS
                Percent_POS = 0.271 # 0.2505

            p.IMU_HEADING_CORRECTION = np.mean(HEADING_EDIT_MEAN)
            # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            PREDICT_VELOCITY_X = (sin(IMU_heading) * ENC_IMU_velocity + LAST_PREDICT_VELOCITY_X)/2 ##GPS_data_heading을 IMU_heading으로 바꿔보고도 실험해주세요!
            PREDICT_VELOCITY_Y = (cos(IMU_heading) * ENC_IMU_velocity + LAST_PREDICT_VELOCITY_Y)/2 ##GPS_data_heading을 IMU_heading으로 바꿔보고도 실험해주세요!

            PREDICT_MOVING_X = PREDICT_VELOCITY_X * ENC_dt
            PREDICT_MOVING_Y = PREDICT_VELOCITY_Y * ENC_dt
            
            if count >DEAD_RECKONING_START:
                PREDICT_INIT_POS[0] += PREDICT_MOVING_X * Percent_POS
                PREDICT_INIT_POS[1] += PREDICT_MOVING_Y * Percent_POS

            PREDICT_POS_X = PREDICT_INIT_POS[0]
            PREDICT_POS_Y = PREDICT_INIT_POS[1]

            PREDICT_POS_X_plot.append(PREDICT_POS_X)
            PREDICT_POS_Y_plot.append(PREDICT_POS_Y)
            GPS_POS_X_plot.append(GPS_POS[0])
            GPS_POS_Y_plot.append(GPS_POS[1])

            LAST_PREDICT_VELOCITY_X = PREDICT_VELOCITY_X
            LAST_PREDICT_VELOCITY_Y = PREDICT_VELOCITY_Y

            SAVE_DATA_ARRAY.append([GPS_POS[0],PREDICT_POS_X,GPS_POS[1],PREDICT_POS_Y,GPS_POS[0]-PREDICT_POS_X,GPS_POS[1]-PREDICT_POS_Y])


        LAST_ENC_velocity = ENC_velocity
        count += 1
        count_plot.append(count)
        if count > 430:
            break
        rate.sleep()

    s.save_data('DEAD_RECKONING_data_120s.csv',SAVE_DATA_ARRAY)
    start = 10   
    plt.figure(1)
    plt.plot(count_plot[start:], GPS_pos_velocity_plot[start:], label='GPS', color='red')
    # plt.plot(count_plot[start:], IMU_velocity_plot[start:], label='IMU', color='blue')
    plt.plot(count_plot[start:], ENC_velocity_plot[start:], label='ENC', color='green')
    plt.plot(count_plot[start:], GPS_data_velocity_plot[start:], label='GPS_DATA', color='orange')   
    plt.plot(count_plot[start:], FILTERED_ENC_VELOCITY_plot[start:], label='ENC_filter', color='black')
    plt.plot(count_plot[start:], EKF_ENC_IMU_plot[start:], label='ENC_IMU', color='magenta')

    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')

    plt.figure(2)
    plt.plot(count_plot[start:], GPS_pos_accel_plot[start:], label='GPS_accel', color='red')
    plt.plot(count_plot[start:], IMU_accel_plot[start:], label='IMU_accel', color='blue')
    # plt.plot(count_plot[start:], ENC_accel_plot[start:], label='ENC_accel', color='green')
    plt.plot(count_plot[start:], GPS_data_accel_plot[start:], label='GPS_DATA_accel', color='orange')
    # plt.plot(count_plot[start:], FILTERED_ENC_VELOCITY_plot[start:], label='ENC_filter_accel', color='black')
    plt.xlabel('count')
    plt.ylabel('accel')
    plt.title('accel')

    plt.figure(3)
    plt.plot(count_plot[start:], GPS_pos_heading_plot[start:], label='GPS_heading', color='red')
    plt.plot(count_plot[start:], IMU_heading_plot[start:], label='IMU_heading', color='blue')
    plt.plot(count_plot[start:], GPS_data_heading_plot[start:], label='GPS_DATA_heading', color='orange')
    # plt.plot(count_plot[start:], FILTERED_ENC_VELOCITY_plot[start:], label='ENC_filter_accel', color='black')
    plt.xlabel('count')
    plt.ylabel('heading')
    plt.title('heading')

    plt.figure(4)
    plt.plot(GPS_POS_X_plot, GPS_POS_Y_plot, label='GPS_POS', color='red')
    plt.plot(PREDICT_POS_X_plot, PREDICT_POS_Y_plot, label='PREDICT_POS', color='magenta')
    plt.scatter(GPS_POS_X_plot[0], GPS_POS_Y_plot[0], label='GPS_POS', color='red')
    plt.scatter(PREDICT_POS_X_plot[0], PREDICT_POS_Y_plot[0], label='PREDICT_POS', color='magenta')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('DEAD_RECKONING')

    plt.show()


if __name__ == "__main__":
    main()                  