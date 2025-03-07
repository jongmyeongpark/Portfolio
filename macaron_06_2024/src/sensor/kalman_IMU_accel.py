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
        
        self.heading_radians = self.heading_radians % (2 * np.pi)
        # if self.heading_radians  < 0:
        #     self.heading_radians  += 2*np.pi

        self.GPS_LAST_VEL_TIME = self.GPS_INPUT_TIME 
        print("----------------------------------------heading",self.speed_kmph ,heading_degrees)

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

                self.GPS_heading = self.GPS_heading % (2 * np.pi)
                # if  self.GPS_heading  < 0:
                #     self.GPS_heading  += 2*np.pi

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
        
        print("__________________________________________________________-",self.linear_x,self.linear_y)
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

        # self.yaw = -self.yaw #######################################################################임시

        self.yaw = self.yaw % (2 * np.pi)
        # if self.yaw  < 0:
            # self.yaw  += 2*np.pi
 

        measurement =[[self.linear_x],[self.linear_y],[self.linear_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.linear_x2 = int(result[0][0] * 10**a) / 10**a
        self.linear_y2 = int(result[1][0] * 10**a) / 10**a
        self.linear_z2 = int(result[2][0] * 10**a) / 10**a

        print("--------------------------------------------------",self.linear_x2,self.linear_y2)


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
             
        return self.IMU_vel_total


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
    measurement_variance = 0.00045  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
    estimation_error = 1.0  # 초기 추정 오차
    initial_value = 0 # 초기 값게

    kf = KalmanFilter(process_variance, measurement_variance, estimation_error, initial_value)

    process_variance_1 = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance_1 = 8.645e-5   # 측정 노이즈 공분산
    estimation_error_1 = 1.0  # 초기 추정 오차
    initial_value_1 = 0 # 초기 값
    kf_1 = KalmanFilter_IMU(process_variance_1, measurement_variance_1, estimation_error_1, initial_value_1)

    # 필터를 적용한 데이터
    filtered_velocities = []
    filtered_velocities_1 = []

    last_timer = time.time()

    Edit_heading_IMU_GPS = p.GPS_heading- p.yaw
    if Edit_heading_IMU_GPS  < 0:
        Edit_heading_IMU_GPS  += 2*np.pi
    print(np.rad2deg(Edit_heading_IMU_GPS))

    # Edit_heading_GPS_GPS = p.GPS_heading- p.heading_radians
    # if Edit_heading_GPS_GPS  < 0:
    #     Edit_heading_GPS_GPS  += 2*np.pi
    # print(np.rad2deg(Edit_heading_GPS_GPS))

    # Edit_heading_IMU_GPS = -3.9
    Edit_heading_GPS_GPS = 0

    meet_heading_vel = 0    
    GPS_sensor_accel = 0

    IMU_accel_plot = []
    GPS_accel_plot = []


    while not rospy.is_shutdown():  
        GPS_vel= round(p.GPS_VELOCITY(),3)
        IMU_vel = round(p.IMU_velocity_calc(),3)
        dt,ENC_vel = p.erp_Velocity()
        count += 1

        ENC_vel = round(ENC_vel,3)
        print(GPS_vel,IMU_vel,ENC_vel)
        
        filtered_value = kf.update(ENC_vel) %90 
        filtered_velocities.append(filtered_value)

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
     
        IMU_heading = p.yaw + Edit_heading_IMU_GPS
        # IMU_heading = p.heading_radians
        # IMU_heading = GPS_heading

        print("-------------------------------",np.rad2deg(IMU_heading),np.rad2deg(IMU_heading))

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

        if p.GPS_VEL_INTERVER != 0:
            GPS_sensor_accel = GPS_sensor_velocity / p.GPS_VEL_INTERVER / 3.6

        IMU_accel = np.sqrt(p.linear_x2**2+p.linear_y2)

        print("--------------------------------------------------------",IMU_accel,GPS_sensor_accel)

        IMU_accel_plot.append(IMU_accel)
        GPS_accel_plot.append(GPS_sensor_accel)


        if count > 600:
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
    plt.xlabel('count')
    plt.ylabel('heading')
    plt.title('heading')    

    plt.show()

        # 측정 노이즈 공분산 행렬 추정
    ENC_plot = np.array(ENC_plot)
    GPS_plot = np.array(GPS_plot)

    measurement_errors = ENC_plot - GPS_sensor_velocity_plot
    R = np.cov(measurement_errors.T)


    print("추정된 측정 노이즈 공분산 행렬 (R):")
    print(R) 
    print("교점 헤딩값",meet_heading_vel)



if __name__ == "__main__":
    main()                          