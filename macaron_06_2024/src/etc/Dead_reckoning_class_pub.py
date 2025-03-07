import numpy as np
from pyquaternion import Quaternion
import rospy
import numpy as np
from scipy import stats

import random

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
from std_msgs.msg import Float64,Float32
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

class Dead_reckoning_class:
    def __init__(self):
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_gps = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.gps_callback,queue_size=1)
        # self.sub_gps_heading = rospy.Subscriber("ublox_gps/navpvt", NavPVT, self.gps_heading_callback,queue_size=1)
        self.sub_gps_vel = rospy.Subscriber("ublox_gps/navpvt",NavPVT,self.gps_data_callback,queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.Dead_reckoning_pub=rospy.Publisher('Dead_reckoning_pose', Point, queue_size=1) # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.Dead_reckoning_pos = Point()

        self.Dead_reckoning_speed_pub=rospy.Publisher('/Dead_reckoning_speed', Float32, queue_size=1) # speed [m/s]


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

        self.Dead_reckoning_signal = 0

        ##################################

        process_variance = 1e-5  # 프로세스 노이즈 공분산
        # measurement_variance = 0.000375  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
        measurement_variance = 0.0001
        estimation_error = 1.0  # 초기 추정 오차
        initial_value = 0 # 초기 값게

        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

        ###################################

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

        ################################

        self.Dead_reckoning_init = 0
        self.ENC_IMU_velocity = 0

        self.GPS_pos_velocity_plot,self.GPS_pos_accel_plot,self.GPS_pos_heading_plot = [],[],[]
        self.GPS_data_velocity_plot,self.GPS_data_accel_plot,self.GPS_data_heading_plot = [],[],[]
        self.IMU_velocity_plot,self.IMU_accel_plot,self.IMU_heading_plot = [],[],[]
        self.ENC_velocity_plot,self.ENC_accel_plot = [],[]

        self.FILTERED_ENC_VELOCITY_plot = []
        self.EKF_ENC_IMU_plot = []
        self.ENC_IMU_velocity_plot = []

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
                Distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                self.GPS_VECOCITY_Heading = np.arctan2(self.pos[1] - self.last_pos[1],self.pos[0] - self.last_pos[0])

                if self.GPS_VELOCITY_dT != 0:
                    self.GPS_VECOCITY_accel = (self.GPS_VELOCITY_LAST_VEL - self.GPS_VELOCITY_VEL) / self.GPS_VELOCITY_dT /3.6

                self.GPS_VELOCITY_VEL = Distance / self.GPS_VELOCITY_dT * 3.6

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.GPS_VELOCITY_LAST_VEL = self.GPS_VELOCITY_VEL

        return self.GPS_VELOCITY_VEL, self.GPS_VECOCITY_accel, self.GPS_VECOCITY_Heading, self.GPS_VELOCITY_dT


    def gps_data_callback(self,nav_msg):
        self.GPS_DATA_dT = abs(self.GPS_INPUT_TIME - self.GPS_DATA_LAST_TIME)
        velocity_ms = nav_msg.gSpeed / 1000
        self.GPS_DATA_velocity_kmh = velocity_ms * 3.6

        ####################################################
        if self.GPS_DATA_velocity_kmh < 0.5 :
            self.GPS_move_flag = 0
        else:  
            self.GPS_move_flag = 1  
        ####################################################

        if self.GPS_DATA_dT != 0:
            self.GPS_DATA_accel = (self.GPS_DATA_velocity_kmh-self.GPS_DATA_LAST_velocity_kmh) / self.GPS_DATA_dT / 3.6

        if self.GPS_move_flag == 1:
            pos_heading_degrees = nav_msg.heading / 1e5
            self.GPS_DATA_pos_heading_radians = np.radians(pos_heading_degrees)

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

        # if self.yaw  < 0:
        #     self.yaw  += 2*np.pi
        
        # self.yaw = 2*pi - self.yaw
        # self.yaw = (self.yaw) % (2*pi)
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
    
    def update_KalmanFilter_ENC(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value    
    
    def predict_EKF_ENC_IMU(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_EKF_ENC_IMU(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = self.P - K @ self.H @ self.P

    def run_EKF_ENC_IMU(self, z, dt):
        self.dt = dt
        self.predict_EKF_ENC_IMU()
        self.update_EKF_ENC_IMU(z)
        return self.x    
    
    def Dead_reckoning(self,signal):
        print("_______________________signal : ",self.Dead_reckoning_signal)
        if self.Dead_reckoning_signal < 3:
            self.Dead_reckoning_signal = signal

        if signal == 4:
            self.Dead_reckoning_signal = 4

        elif signal == 5:
            self.Dead_reckoning_init = 0    

        if self.Dead_reckoning_init == 0:
            GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = 0,0,0,0
            GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = 0,0,0,0
            IMU_velocity,IMU_accel,IMU_heading,IMU_dt = 0,0,0,0
            ENC_velocity,ENC_accel,ENC_LOCAL_heading,ENC_dt,ENC_flag = 0,0,0,0,0

            self.IMU_count = 3
            for i in range(self.IMU_count):
                IMU_velocity,IMU_accel,IMU_heading,IMU_dt = self.IMU_VELOCITY()
                self.IMU_heading_plot.append(IMU_heading)

            FILTERED_ENC_VELOCITY = 0

            EKF_ENC_IMU_VEL = 0
            self.LAST_EKF_ENC_IMU_VEL = 0

            IMU_Correction = 0

            self.LAST_ENC_velocity = 0
            self.ENC_IMU_velocity = 0

            self.ENC_FILTER = [0,0,0]

            GPS_POS = [0,0]
            self.GPS_POS_X_plot= []
            self.GPS_POS_Y_plot = []

            self.PREDICT_INIT_POS = [0,0]
            self.PREDICT_POS_X_plot = []
            self.PREDICT_POS_Y_plot = []
            
            PREDICT_POS_X = 0
            PREDICT_POS_Y = 0
            PREDICT_MOVING_X = 0
            PREDICT_MOVING_Y = 0

            PREDICT_VELOCITY_X = 0
            PREDICT_VELOCITY_Y = 0
            self.LAST_PREDICT_VELOCITY_X = 0
            self.LAST_PREDICT_VELOCITY_Y = 0

            self.HEADING_EDIT = 0
            self.count,self.count_plot = 0,[]
            self.HEADING_EDIT_array = []

            self.DEAD_RECKOING_HEAIDNG_plot = [0,0,0,0]
            self.DEAD_RECKOING_count_plot = []

            self.EKF_ENC_IMU_VEL = 0

            self.Dead_reckoning_init = 1

        elif self.Dead_reckoning_init == 1:
            GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = self.GPS_VELOCITY()
            GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = self.GPS_DATA_VELOCITY()
            IMU_velocity,IMU_accel,IMU_heading,IMU_dt = self.IMU_VELOCITY()
            ENC_velocity,ENC_accel,ENC_LOCAL_heading,ENC_dt,ENC_flag = self.ENC_VELOCITY() 
            
            if GPS_data_velocity < 0.3 and IMU_accel > 0:
                IMU_Correction = self.IMU_CORRECTION()

            else:
                IMU_Correction = 0

            IMU_accel = IMU_accel + IMU_Correction

            if ENC_flag == 1 or IMU_accel > 0:
                self.IMU_VELOCITY_VEL = ENC_velocity
                self.ENC_IMU_velocity = ENC_velocity
            else:
                self.ENC_IMU_velocity = self.ENC_IMU_velocity + IMU_accel*IMU_dt

            if ENC_velocity == 0:
                self.ENC_IMU_velocity = 0

            if self.ENC_IMU_velocity < 0:
                self.ENC_IMU_velocity = 0   

            FILTERED_ENC_VELOCITY = self.update_KalmanFilter_ENC(self.ENC_IMU_velocity)
            self.FILTERED_ENC_VELOCITY_plot.append(FILTERED_ENC_VELOCITY)

        ################################################################################################
            if ENC_velocity > 20:
                ENC_velocity = 0

            if ENC_velocity != self.LAST_ENC_velocity: 
                self.ENC_FILTER[0] = self.ENC_FILTER[1]
                self.ENC_FILTER[1] = self.ENC_FILTER[2]
                self.ENC_FILTER[2] = ENC_velocity

                if (self.ENC_FILTER[1] > self.ENC_FILTER[0] and self.ENC_FILTER[1] > self.ENC_FILTER[2]) or (self.ENC_FILTER[1] < self.ENC_FILTER[0] and self.ENC_FILTER[1] < self.ENC_FILTER[2]):
                    self.ENC_FILTER[1] = (self.ENC_FILTER[0] + self.ENC_FILTER[2])/2  
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    FILTERED_ENC_velocity = self.ENC_FILTER[1]

                else:
                    FILTERED_ENC_velocity = self.ENC_FILTER[1]

            LAST_ENC_velocity = ENC_velocity

            obs = [FILTERED_ENC_VELOCITY,IMU_accel]
            z = np.array(obs).reshape(-1, 1)
            estimated_state = self.run_EKF_ENC_IMU(z,IMU_dt)
            EKF_ENC_IMU_VEL = estimated_state[1][0]

            Percent = 0.8
            if IMU_accel>0:
                if EKF_ENC_IMU_VEL > self.LAST_EKF_ENC_IMU_VEL + IMU_accel*IMU_dt*Percent:
                    EKF_ENC_IMU_VEL = self.LAST_EKF_ENC_IMU_VEL 

            else:
                if EKF_ENC_IMU_VEL < self.LAST_EKF_ENC_IMU_VEL + IMU_accel*IMU_dt*Percent:
                    EKF_ENC_IMU_VEL = self.LAST_EKF_ENC_IMU_VEL    

            if EKF_ENC_IMU_VEL < 0:
                EKF_ENC_IMU_VEL = 0

            self.EKF_ENC_IMU_VEL = EKF_ENC_IMU_VEL
            self.Dead_reckoning_speed_pub.publish(self.EKF_ENC_IMU_VEL)

            self.EKF_ENC_IMU_plot.append(EKF_ENC_IMU_VEL)
            self.LAST_EKF_ENC_IMU_VEL = estimated_state[1][0]

            self.GPS_pos_velocity_plot.append(GPS_pos_velocity)
            self.GPS_data_velocity_plot.append(GPS_data_velocity)
            self.IMU_velocity_plot.append(IMU_velocity)
            self.ENC_velocity_plot.append(ENC_velocity)
            self.ENC_IMU_velocity_plot.append(self.ENC_IMU_velocity)   

            self.GPS_pos_accel_plot.append(GPS_pos_accel)
            self.GPS_data_accel_plot.append(GPS_data_accel)
            self.IMU_accel_plot.append(IMU_accel)
            self.ENC_accel_plot.append(ENC_accel)

            self.GPS_pos_heading_plot.append(GPS_pos_heading)
            self.GPS_data_heading_plot.append(GPS_data_heading)
            self.IMU_heading_plot.append(IMU_heading)

            # IMU_heading = IMU_heading_plot[-1] + random.uniform(-0.1, 0.1) 
            IMU_heading = self.IMU_heading_plot[-1]
            #####################################
            DEAD_RECKONING_START = 200
            if self.GPS_move_flag == 1 and self.Dead_reckoning_signal == 1:
                if GPS_data_heading > IMU_heading:
                        self.HEADING_EDIT = GPS_data_heading - IMU_heading
                        print("HEADING_DATA_GET_A",self.HEADING_EDIT,IMU_heading,GPS_data_heading)
                elif GPS_data_heading < IMU_heading:
                    self.HEADING_EDIT = IMU_heading - GPS_data_heading
                    print("HEADING_DATA_GET_B",self.HEADING_EDIT,IMU_heading,GPS_data_heading)

                self.HEADING_EDIT_array.append(self.HEADING_EDIT)    
                self.HEADING_EDIT_MEAN = np.mean(self.HEADING_EDIT_array)
            #####################################

            if self.Dead_reckoning_signal >=2:
                print("Dead_Reckoning...",self.Dead_reckoning_signal,self.count)
                GPS_POS = self.GPS_POS()
                if self.Dead_reckoning_signal == 2:
                    self.PREDICT_INIT_POS = GPS_POS
                    self.Percent_POS = 0.2698 # 0.271 

                # IMU_heading = IMU_heading +random.uniform(-0.05, 0.05) 
                self.DEAD_RECKOING_HEAIDNG_plot.append(IMU_heading)
                self.DEAD_RECKOING_count_plot.append(self.count)
                self.IMU_HEADING_CORRECTION = np.mean(self.HEADING_EDIT_MEAN)
                PREDICT_VELOCITY_X = (sin(IMU_heading) * self.EKF_ENC_IMU_VEL + self.LAST_PREDICT_VELOCITY_X)/2 ##GPS_data_heading을 IMU_heading으로 바꿔보고도 실험해주세요!
                PREDICT_VELOCITY_Y = (cos(IMU_heading) * self.EKF_ENC_IMU_VEL + self.LAST_PREDICT_VELOCITY_Y)/2 ##GPS_data_heading을 IMU_heading으로 바꿔보고도 실험해주세요!

                PREDICT_MOVING_X = PREDICT_VELOCITY_X * ENC_dt
                PREDICT_MOVING_Y = PREDICT_VELOCITY_Y * ENC_dt
                
                if self.Dead_reckoning_signal == 3:
                    self.PREDICT_INIT_POS[0] += PREDICT_MOVING_X * self.Percent_POS
                    self.PREDICT_INIT_POS[1] += PREDICT_MOVING_Y * self.Percent_POS

                    self.Dead_reckoning_pos.x = self.PREDICT_INIT_POS[0]
                    self.Dead_reckoning_pos.y = self.PREDICT_INIT_POS[1]
                    self.Dead_reckoning_pos.z = IMU_heading
                    self.Dead_reckoning_pub.publish(self.Dead_reckoning_pos)

                PREDICT_POS_X = self.PREDICT_INIT_POS[0]
                PREDICT_POS_Y = self.PREDICT_INIT_POS[1]

                self.PREDICT_POS_X_plot.append(PREDICT_POS_X)
                self.PREDICT_POS_Y_plot.append(PREDICT_POS_Y)
                self.GPS_POS_X_plot.append(GPS_POS[0])
                self.GPS_POS_Y_plot.append(GPS_POS[1])

                self.LAST_PREDICT_VELOCITY_X = PREDICT_VELOCITY_X
                self.LAST_PREDICT_VELOCITY_Y = PREDICT_VELOCITY_Y


            LAST_ENC_velocity = ENC_velocity 
            if self.Dead_reckoning_signal == 2:
                self.Dead_reckoning_signal += 1
            self.count += 1
            self.count_plot.append(self.count)
            if self.Dead_reckoning_signal == 4:
                start = 10   
                plt.figure(1)
                plt.plot(self.count_plot[start:], self.GPS_pos_velocity_plot[start:], label='GPS', color='red')
                # plt.plot(count_plot[start:], IMU_velocity_plot[start:], label='IMU', color='blue')
                plt.plot(self.count_plot[start:], self.ENC_velocity_plot[start:], label='ENC', color='green')
                plt.plot(self.count_plot[start:], self.GPS_data_velocity_plot[start:], label='GPS_DATA', color='orange')   
                plt.plot(self.count_plot[start:], self.FILTERED_ENC_VELOCITY_plot[start:], label='self.ENC_filter', color='black')
                plt.plot(self.count_plot[start:], self.EKF_ENC_IMU_plot[start:], label='ENC_IMU', color='magenta')

                plt.xlabel('count')
                plt.ylabel('velocity')
                plt.title('Velocity')

                plt.figure(2)
                plt.plot(self.count_plot[start:], self.GPS_pos_accel_plot[start:], label='GPS_accel', color='red')
                plt.plot(self.count_plot[start:], self.IMU_accel_plot[start:], label='IMU_accel', color='blue')
                # plt.plot(count_plot[start:], ENC_accel_plot[start:], label='ENC_accel', color='green')
                plt.plot(self.count_plot[start:], self.GPS_data_accel_plot[start:], label='GPS_DATA_accel', color='orange')
                # plt.plot(count_plot[start:], FILTERED_ENC_VELOCITY_plot[start:], label='self.ENC_filter_accel', color='black')
                plt.xlabel('count')
                plt.ylabel('accel')
                plt.title('accel')

                plt.figure(3)
                # plt.plot(count_plot[start:], GPS_pos_heading_plot[start:], label='GPS_heading', color='red')
                plt.plot(self.count_plot[start:], self.IMU_heading_plot[start:-self.IMU_count], label='IMU_heading', color='blue')
                plt.plot(self.count_plot[start:], self.GPS_data_heading_plot[start:], label='GPS_DATA_heading', color='orange')
                plt.plot(self.DEAD_RECKOING_count_plot[:], self.DEAD_RECKOING_HEAIDNG_plot[:-(4)], label='self.ENC_filter_accel', color='black')
                plt.xlabel('count')
                plt.ylabel('heading')
                plt.title('heading')

                plt.figure(4)
                plt.plot(self.GPS_POS_X_plot, self.GPS_POS_Y_plot, label='GPS_POS', color='red')
                plt.plot(self.PREDICT_POS_X_plot, self.PREDICT_POS_Y_plot, label='PREDICT_POS', color='magenta')
                plt.scatter(self.GPS_POS_X_plot[0], self.GPS_POS_Y_plot[0], label='GPS_POS', color='red')
                plt.scatter(self.PREDICT_POS_X_plot[0], self.PREDICT_POS_Y_plot[0], label='PREDICT_POS', color='magenta')
                plt.xlabel('X')
                plt.ylabel('Y')
                plt.title('DEAD_RECKONING')

                plt.show()

        

def main():
    rospy.init_node('mapping', anonymous=True)
    D = Dead_reckoning_class()
    rate = rospy.Rate(10)

    D.Dead_reckoning(0)
    start = time.time()
    while not rospy.is_shutdown():    
        timer = abs(time.time() - start)
        if timer >= 10 and timer < 20:
            D.Dead_reckoning(1) ##Dead_reckoning()만 실행시키면 됨 // 숫자 1은 헤딩값 보정, 2는 데드렉코닝 시작, 4는 데드렉코닝 종료 및 그래프(생략 예정), 5는 데드레코닝 초기화
        elif timer >= 20 and timer < 50: #음영 구역 진입 전 여유롭게 1를 시작해야하며, 음영구역 시작 전 2를 시작해야함
            D.Dead_reckoning(2)
        elif timer >= 50:
            D.Dead_reckoning(4)  
        else:
            D.Dead_reckoning(0) 
        rate.sleep()
    
if __name__ == "__main__":
    main()  
