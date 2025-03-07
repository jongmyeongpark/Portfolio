import numpy as np
from pyquaternion import Quaternion
import rospy
import numpy as np

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
        self.sub_gps_vel = rospy.Subscriber("ublox_gps/navpvt",NavPVT,self.gps_vel_callback,queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.pos = [0,0]
        self.last_pos = [0,0]
        self.GPS_init_Count = 0
        self.GPS_heading = 0
        self.velocity = 0
        
        self.IMU_init_Count = 0
        self.IMU_last_accel_x = 0
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z= 0
        self.IMU_last_Time  = time.time()
        self.IMU_input_Time = time.time()
        self.IMU_vel_total= 0
        self.last_velocity = 0
        self.Time_interval = 0

        self.timer = 0.1
        self.count = 0
        self.yaw  = 0
        self.pitch = 0
        self.linear_x2  = 0
        self.linear_y2= 0
        self.linear_z2 = 0
        self.IMU_vel_x =0

        self.IMU_accel_total = 0
        self.IMU_accel_x = 0
        self.IMU_accel_y = 0
        self.last_linear_x2 = 0
        self.last_linear_y2 = 0
        self.IMU_accel_count_avg = 0
 

        self.erp_init_Count= 0
        self.last_enc = 0
        self.enc = 0 
        self.erp_velocity2  = 0
        self.erp_INPUT_TIME = time.time()
        self.erp_LAST_TIME = time.time()
        self.dt = 0

        self.heading_radians = 0
        self.speed_kmph = 0

        self.GPS_LAST_VEL_TIME = time.time()
        self.GPS_VEL_INTERVER = 0     
        self.ENC_ACCEL_TIMER = time.time()   


    # def gps_heading_callback(self, heading_msg):
    #     heading_degrees = heading_msg.heading / 1_000_000
    #     # 도를 라디안으로 변환
    #     heading_radians = np.radians(heading_degrees)
    #     print("----------------------------------------heading",heading_degrees)

    #     return heading_radians
        
    def gps_vel_callback(self, vel_msg):
        self.GPS_VEL_INTERVER = abs(self.GPS_INPUT_TIME - self.GPS_LAST_VEL_TIME)
        
        speed_mps = vel_msg.gSpeed / 1000.0
        # m/s to km/h
        self.speed_kmph  = speed_mps * 3.6

        heading_degrees = vel_msg.heading / 1e5
        # 도를 라디안으로 변환
        self.heading_radians = np.radians(heading_degrees)
        
        while self.heading_radians  > 2*np.pi:
            self.heading_radians  -= 2*np.pi

        self.GPS_LAST_VEL_TIME = self.GPS_INPUT_TIME 
        # print("----------------------------------------heading",self.speed_kmph ,heading_degrees)

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC
        self.erp_INPUT_TIME = time.time()

    def erp_Velocity(self):
        if self.erp_init_Count == 0:
           
            self.erp_LAST_TIME = self.erp_INPUT_TIME
            self.last_enc = self.enc  
            self.erp_count = 0
            self.erp_init_Count = 1    
      
        elif self.erp_init_Count == 1:    
            if self.last_enc !=  self.enc:
                self.dt = abs(self.erp_INPUT_TIME-self.erp_LAST_TIME)
                self.erp_velocity2 = 0.5*abs(self.enc - self.last_enc)*(2*np.pi/100)/2/self.dt * 3.6  # 바퀴 지름 50cm
                if self.erp_velocity2 >50:
                    self.erp_velocity2 = 0
                self.erp_LAST_TIME = self.erp_INPUT_TIME
                self.last_enc = self.enc 
                self.erp_count = 0
                self.ENC_ACCEL_TIMER = time.time()


        return self.dt,self.erp_velocity2 
    
    def gps_callback(self, gps_msg):
        
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()
        if self.count < 5:
            self.count += 1
        
    def GPS_VELOCITY(self):
        if self.GPS_init_Count == 0:
           
            self.GPS_LAST_TIME = self.GPS_INPUT_TIME
            self.last_pos = self.pos  
            if self.count > 4:
                # print(self.count)
                self.GPS_init_Count = 1
              
        elif self.GPS_init_Count == 1:    
            self.Time_interval = abs(self.GPS_INPUT_TIME- self.GPS_LAST_TIME)
            if self.Time_interval > self.timer:
                distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                
                self.GPS_heading = np.arctan2(self.pos[1] - self.last_pos[1],self.pos[0] - self.last_pos[0])

                # self.GPS_heading = self.GPS_heading % (2 * np.pi)
                if  self.GPS_heading  > 2*np.pi:
                    self.GPS_heading  -= 2*np.pi

                self.velocity = distance/self.Time_interval *3.6
                

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.last_velocity = self.velocity

        return self.velocity

    def GPS_POS(self):
        return self.pos       
        
    def imu_callback(self, imu): #임시로 마이너스
        self.linear_x = imu.linear_acceleration.x
        self.linear_y = imu.linear_acceleration.y 
        self.linear_z = imu.linear_acceleration.z 
        self.angular_z = imu.angular_velocity.z 
        self.magnetic_field_x = imu.orientation.x
        self.magnetic_field_y = imu.orientation.y
        self.magnetic_field_z = imu.orientation.z
        self.IMU_input_Time = time.time()
        
        # print("__________________________________________________________-",self.linear_x,self.linear_y)
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

        # self.yaw = -self.yaw #######################################################################임시

        while self.yaw  > 2*np.pi:
            self.yaw  -= 2*np.pi
 

        measurement =[[self.linear_x],[self.linear_y],[self.linear_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.linear_x2 = -int(result[0][0] * 10**a) / 10**a
        self.linear_y2 = int(result[1][0] * 10**a) / 10**a
        self.linear_z2 = int(result[2][0] * 10**a) / 10**a

        self.IMU_accel_x = (self.linear_x2+self.last_linear_x2)/2 
        self.IMU_accel_y = (self.linear_y2+self.last_linear_y2)/2 

        self.last_linear_x2,self.last_linear_y2 = self.linear_x2 ,self.linear_y2 
        
        i = 1
        if self.IMU_accel_x >= 0:
            i = 1
        else:
            i = -1
        # self.IMU_accel_total += np.sqrt(self.IMU_accel_x**2+ self.IMU_accel_y**2)*i
        # self.IMU_accel_total += self.IMU_accel_x 
        self.IMU_accel_total = self.IMU_accel_x
        # print(self.IMU_accel_total )

        # print("--------------------------------------------------",self.linear_x2,self.linear_y2)
        self.IMU_accel_count_avg += 1

    def IMU_HEADING(self):
        return self.yaw        
            
    def IMU_velocity_calc(self):
        if self.IMU_init_Count == 0:
            
            self.IMU_last_accel_x = self.linear_x2
            self.IMU_last_accel_y = self.linear_y2
            self.IMU_last_Time  = time.time()
            self.IMU_init_Count = 1
        
            
        elif self.IMU_init_Count == 1:
            IMU_time_intervel = self.IMU_input_Time -self.IMU_last_Time 
            if IMU_time_intervel >= 0.001:  
                self.IMU_vel_x = (self.linear_x2+self.IMU_last_accel_x)*IMU_time_intervel/2 
                self.IMU_vel_y = (self.linear_y2+self.IMU_last_accel_y) * IMU_time_intervel/2 
                
                i = 1
                if self.IMU_vel_x > 0:
                    i = 1
                elif self.IMU_vel_x == 0:
                    i = 0
                else:
                    i = -1    
                # self.IMU_vel_total += i*np.sqrt(self.IMU_vel_x**2+self.IMU_vel_y**2) * 3.6
                self.IMU_vel_total += self.IMU_vel_x * 3.6
                
                if self.IMU_vel_total == 0:
                    self.vel_error = 0
                if self.IMU_vel_total<0:
                    self.IMU_vel_total = 0  
                    self.vel_error = 0

                self.IMU_last_Time = self.IMU_input_Time
                self.IMU_last_accel_x  = self.linear_x2
                self.IMU_last_accel_y  = self.linear_y2

        self.IMU_accel_count = 1        

        return self.IMU_vel_total, self.IMU_accel_count


class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value
    

class KalmanFilter_IMU:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement updateself.ENC_ACCEL_TIMER = time.time()

class KalmanFilter_GPS:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value    
    

class KalmanFilter_ENC_ACCEL:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value      

def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    rate = rospy.Rate(10)

    init_pos = [0,0]
    while init_pos == [0,0]:
        init_pos = p.GPS_POS()

    GPS_vel= 0
    IMU_vel = 0
    ENC_vel = 0

    count_plot = []
    GPS_plot = []
    IMU_plot = []
    ENC_plot = []

    GPS_X_plot = []
    GPS_Y_plot = []

    Predict_X_plot = []
    Predict_Y_plot = []

    GPS_pos = [0,0]
    XY_ENC_vel = [0,0]
    XY_ENC_POS = [0,0]
    last_ENC_VEL = 0
    X_last_ENC_VEL = 0
    Y_last_ENC_VEL = 0

    GPS_heading_plot = []
    IMU_heading_plot = []

    GPS_sensor_heading_plot = []
    GPS_sensor_velocity_plot = []

    X_GPS_PLOT = [] #데드렉코닝 정확성 테스트
    Y_GPS_PLOT = []
    init_pos_GPS = [0,0]
    init_pos_GPS = init_pos
    X_last_GPS_VEL = 0
    Y_last_GPS_VEL = 0


    count = 0

    # 칼만 필터 초기화
    process_variance = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance = 0.000375  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
    estimation_error = 1.0  # 초기 추정 오차
    initial_value = 0 # 초기 값게

    kf = KalmanFilter(process_variance, measurement_variance, estimation_error, initial_value)

    process_variance_1 = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance_1 = 8.645e-5   # 측정 노이즈 공분산
    estimation_error_1 = 1.0  # 초기 추정 오차
    initial_value_1 = 0 # 초기 값
    kf_1 = KalmanFilter_IMU(process_variance_1, measurement_variance_1, estimation_error_1, initial_value_1)

    process_variance_2 = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance_2 =  0.00001  # 측정 노이즈 공분산
    estimation_error_2 = 1.0  # 초기 추정 오차
    initial_value_2 = 0 # 초기 값
    kf_2 = KalmanFilter_GPS(process_variance_2, measurement_variance_2, estimation_error_2, initial_value_2)

    process_variance_3 = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance_3 =  0.00001  # 측정 노이즈 공분산
    estimation_error_3 = 1.0  # 초기 추정 오차
    initial_value_3 = 0 # 초기 값
    kf_3 = KalmanFilter_ENC_ACCEL(process_variance_3, measurement_variance_3, estimation_error_3, initial_value_3)

    # 필터를 적용한 데이터
    filtered_velocities = []
    filtered_velocities_1 = []

    last_timer = time.time()

    # Edit_heading_IMU_GPS = -p.GPS_heading+p.yaw
    # while Edit_heading_IMU_GPS  > 2*np.pi:
    #     Edit_heading_IMU_GPS  -= 2*np.pi
    # print(np.rad2deg(Edit_heading_IMU_GPS))

    Edit_heading_GPS_GPS = p.GPS_heading- p.heading_radians
    while Edit_heading_GPS_GPS  > 2*np.pi:
        Edit_heading_GPS_GPS  -= 2*np.pi
    # print(np.rad2deg(Edit_heading_GPS_GPS))

    # Edit_heading_IMU_GPS = -3.9
    Edit_heading_IMU_GPS = 0
    Edit_heading_GPS_GPS = 0

    meet_heading_vel = 0    
    GPS_sensor_accel = 0
    GPS_sensor_accel_last = 0
    GPS_sensor_velocity_last = 0

    IMU_accel_plot = []
    GPS_accel_plot = []

    dif_accel = []
    dif_heading = []
    IMU_accel = 0
    count_GPS_ACCEL = 0
    IMU_GPS_ACCEL_EDIT = 0
    last_GPS_TIMM = time.time()

    filtered_accel_2 = []
    filtered_value_2 = 0

    last_ENC_TIME = time.time()
    last_ENC_VEL = 0
    ENC_ACCEL_PLOT = []
    FILTERED_ENC_ACCEL = 0
    FILTERED_ENC_ACCEL_plot = []

    

    while not rospy.is_shutdown():  
        GPS_vel= round(p.GPS_VELOCITY(),3)
        IMU_vel,accel_count = p.IMU_velocity_calc()
        # if accel_count == 1 and p.IMU_accel_count_avg != 0:
        #     IMU_accel = p.IMU_accel_total / p.IMU_accel_count_avg
        #     p.IMU_accel_total = 0
        #     p.IMU_accel_count_avg = 0
        #     accel_count = 0
        IMU_accel = p.IMU_accel_total
        IMU_vel = round(IMU_vel,3)
        IMU_accel = round(IMU_accel,3) + IMU_GPS_ACCEL_EDIT

        dt,ENC_vel = p.erp_Velocity()
        count += 1

        ENC_vel = round(ENC_vel,3)
        # print(GPS_vel,IMU_vel,ENC_vel)
        
        NOW_ENC_TIME = p.ENC_ACCEL_TIMER
        filtered_value = kf.update(ENC_vel)
        filtered_velocities.append(filtered_value)
        


    
        ENC_VEL_INTERVER = abs(NOW_ENC_TIME- last_ENC_TIME)

        if ENC_VEL_INTERVER != 0:
            ENC_ACCEL = (filtered_value-last_ENC_VEL) / 3.6/ ENC_VEL_INTERVER

            FILTERED_ENC_ACCEL = kf_3.update(ENC_ACCEL)

        last_ENC_TIME =  NOW_ENC_TIME
        last_ENC_VEL = filtered_value

        ENC_ACCEL_PLOT.append(ENC_ACCEL)
        FILTERED_ENC_ACCEL_plot.append(FILTERED_ENC_ACCEL)


        
        count_plot.append(count)
        GPS_plot.append(GPS_vel)
        IMU_plot.append(IMU_vel)
        ENC_plot.append(ENC_vel)

        if ENC_vel <1:
            filtered_value = 0

        # if p.linear_x2 >= 0:
        #     time_interval = time.time() - last_timer
        #     if time_interval > 0.1:
        #         p.IMU_vel_total = filtered_value
        #         last_timer = time.time()

        # if p.linear_x2 >= 0:
        time_interval = time.time() - last_timer
        if time_interval > 0.01:
            p.IMU_vel_total = filtered_value
            last_timer = time.time()        


        # filtered_value_1 = kf_1.update(p.IMU_vel_x)
        filtered_value_1 = kf_1.update(IMU_vel)
        filtered_velocities_1.append(filtered_value_1)

        GPS_pos = p.GPS_POS()
        GPS_heading = p.GPS_heading

        Edit_heading_IMU_GPS_1 = p.yaw + Edit_heading_IMU_GPS
        while Edit_heading_IMU_GPS_1 > 2*np.pi:
            Edit_heading_IMU_GPS_1 -= 2*np.pi
        IMU_heading =  Edit_heading_IMU_GPS_1
        # IMU_heading = p.heading_radians
        # IMU_heading = GPS_heading

        # print("-------------------------------",np.rad2deg(IMU_heading),np.rad2deg(IMU_heading))

        ENC_vel_ms = filtered_value / 3.6
        X_ENC_vel = ENC_vel_ms * np.cos(IMU_heading)
        Y_ENC_vel = ENC_vel_ms * np.sin(IMU_heading)

        plus_X_ENC_vel = (X_ENC_vel+X_last_ENC_VEL)*dt/2
        plus_Y_ENC_vel = (Y_ENC_vel+Y_last_ENC_VEL)*dt/2

        init_pos[0] += plus_X_ENC_vel
        init_pos[1] += plus_Y_ENC_vel

        XY_ENC_POS = [init_pos[0],init_pos[1]]

        # XY_ENC_vel = []
        X_last_ENC_VEL = X_ENC_vel
        Y_last_ENC_VEL = Y_ENC_vel

        GPS_X_plot.append(GPS_pos[0])
        GPS_Y_plot.append(GPS_pos[1])

        Predict_X_plot.append(XY_ENC_POS[0])
        Predict_Y_plot.append(XY_ENC_POS[1])


        GPS_heading_plot.append(GPS_heading)
        IMU_heading_plot.append(IMU_heading)


        GPS_sensor_velocity,GPS_sensor_heading = p.speed_kmph ,p.heading_radians+Edit_heading_GPS_GPS

        GPS_sensor_heading_plot.append(GPS_sensor_heading)
        GPS_sensor_velocity_plot.append(GPS_sensor_velocity)

        if GPS_sensor_heading >= GPS_heading - 0.5 and GPS_sensor_heading <= GPS_heading + 0.5:
            meet_heading_vel = GPS_heading

        # count_GPS_ACCEL += 1
        # GPS_VEL_TIMER += p.GPS_VEL_INTERVER
        # if p.GPS_VEL_INTERVER != 0:
        
        
        if p.GPS_VEL_INTERVER != 0:
            GPS_sensor_accel = (GPS_sensor_velocity-GPS_sensor_velocity_last) / (p.GPS_VEL_INTERVER * 3.6)
            # GPS_sensor_accel = (GPS_sensor_velocity-GPS_sensor_velocity_last) / (GPS_VEL_INTERVER * 3.6)

            filtered_value_2 = kf_2.update(GPS_sensor_accel)

        print(filtered_value_2)
        if GPS_sensor_accel > 2 or filtered_value_2>2:
            GPS_sensor_accel = 0
            filtered_value_2 = 0

        GPS_sensor_velocity_last = GPS_sensor_velocity


        # IMU_accel = np.sqrt(p.linear_x2**2+p.linear_y2)
        # IMU_accel = p.linear_x2
        print("--------------------------------------------------------",IMU_accel,GPS_sensor_accel)

        IMU_accel_plot.append(IMU_accel)
        GPS_accel_plot.append(GPS_sensor_accel)
        filtered_accel_2.append(filtered_value_2)

        dif_accel.append(GPS_sensor_accel-IMU_accel)
        dif_heading.append(GPS_heading-IMU_heading)

        GPS_sensor_accel_last = GPS_sensor_accel
        p.IMU_accel_total  =0

        # if count == 2:
        #     IMU_GPS_ACCEL_EDIT = GPS_sensor_accel - IMU_accel
        #     print("------------------------------------",IMU_GPS_ACCEL_EDIT)
        

        if count > 810:
            break
        rate.sleep()
        
 
    start = 5    
    plt.figure(1)
    plt.plot(count_plot[start:], IMU_plot[start:], label='IMU', color='blue')
    plt.plot(count_plot[start:], ENC_plot[start:], label='ENC', color='green')
    plt.plot(count_plot[start:], GPS_plot[start:], label='GPS', color='red')
    plt.plot(count_plot[start:], filtered_velocities[start:], label='ENC_filter', color='black')
    plt.plot(count_plot[start:], GPS_sensor_velocity_plot[start:], label='GPS_sensor', color='orange')
    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')

    # plt.figure(2)
    # plt.plot(count_plot[start:], filtered_velocities_1[start:], label='IMU', color='blue')
    # plt.xlabel('count')
    # plt.ylabel('accel')
    # plt.title('accel')     

    plt.figure(2)
    plt.plot(GPS_X_plot[start:], GPS_Y_plot[start:], label='GPS', color='red')
    # plt.plot(X_GPS_PLOT[start:], Y_GPS_PLOT[start:], label='GPS_test', color='orange')
    plt.plot(Predict_X_plot[start:], Predict_Y_plot[start:], label='ENC', color='black')
    plt.xlabel('count')
    plt.ylabel('position')
    plt.title('position')

    plt.figure(3)
    plt.plot(count_plot[start:], GPS_heading_plot[start:], label='GPS_heading', color='red')
    plt.plot(count_plot[start:], GPS_sensor_heading_plot[start:], label='GPS_sensor_heading', color='orange')
    plt.plot(count_plot[start:], IMU_heading_plot[start:], label='IMU_heading', color='black')
    plt.xlabel('count')
    plt.ylabel('heading')
    plt.title('heading')    

    plt.figure(4)
    plt.plot(count_plot[start:], GPS_accel_plot[start:], label='GPS_accel', color='red')
    plt.plot(count_plot[start:], IMU_accel_plot[start:], label='IMU_accel', color='blue')
    plt.plot(count_plot[start:], filtered_accel_2[start:], label='filtered_GPS_accel', color='orange')
    plt.plot(count_plot[start:], ENC_ACCEL_PLOT[start:], label='ENC_accel', color='green')
    plt.plot(count_plot[start:], FILTERED_ENC_ACCEL_plot[start:], label='filtered_ENC_accel', color='black')
    plt.xlabel('count')
    plt.ylabel('accel')
    plt.title('accel')    

    plt.figure(5)
    plt.plot(count_plot[start:], dif_accel[start:], label='accel_dif', color='red')
    plt.plot(count_plot[start:], dif_heading[start:], label='heading_dif', color='blue')
    plt.xlabel('count')
    plt.ylabel('heading')
    plt.title('차이')   

    plt.show()

        # 측정 노이즈 공분산 행렬 추정
    ENC_plot = np.array(ENC_plot)
    GPS_plot = np.array(GPS_plot)

    measurement_errors = ENC_plot - GPS_sensor_velocity_plot
    measurement_errors2 =  GPS_sensor_velocity_plot - ENC_plot
    # R = np.cov(measurement_errors.T)
    covariance_matrix = np.cov(measurement_errors, measurement_errors2,ddof=0)
    print("Sensor 1 and Sensor 2 Error Covariance:", covariance_matrix[0, 1])

    # print("추정된 측정 노이즈 공분산 행렬 (R):")
    # print(R) 
    print("교점 헤딩값",meet_heading_vel)

    print(dif_accel,"accel")
    print(dif_heading,"heading")



if __name__ == "__main__":
    main()                          