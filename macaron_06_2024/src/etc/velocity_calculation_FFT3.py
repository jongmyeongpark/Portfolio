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
from ublox_msgs.msg import NavPVT
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
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.pos = [0,0]
        self.last_pos = [0,0]
        self.GPS_init_Count = 0

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


        self.timer = 0.3
        self.count = 0
        self.yaw  = 0
        self.pitch = 0
        self.linear_x2  = 0
        self.linear_y2= 0
        self.linear_z2 = 0

        self.erp_init_Count= 0
        self.last_enc = 0
        self.enc = 0 
        self.erp_velocity2  = 0
        self.erp_INPUT_TIME = time.time()
        self.erp_LAST_TIME = time.time()


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
            # print(self.erp_count)
            
            # if self.last_enc !=  self.enc:
            if self.last_enc !=  self.enc:
                dt = abs(self.erp_INPUT_TIME-self.erp_LAST_TIME)
                self.erp_velocity2 = 0.5*abs(self.enc - self.last_enc)*(2*np.pi/100)/2/dt * 3.6  # 바퀴 지름 50cm
                if self.erp_velocity2 >50:
                    self.erp_velocity2 = 0
                self.erp_LAST_TIME = self.erp_INPUT_TIME
                self.last_enc = self.enc 
                self.erp_count = 0
                # print(self.erp_velocity2)


        return self.erp_velocity2
    
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
            Time_interval = abs(self.GPS_INPUT_TIME- self.GPS_LAST_TIME)
            if Time_interval > self.timer:
                distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                self.velocity = round(distance/Time_interval *3.6,1)
                

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.last_velocity = self.velocity

        return self.velocity

    def GPS_POS(self):
        return self.pos       
        
    def imu_callback(self, imu):
        self.linear_x = imu.linear_acceleration.x
        self.linear_y = imu.linear_acceleration.y 
        self.linear_z = imu.linear_acceleration.z 
        self.angular_z = imu.angular_velocity.z 
        self.magnetic_field_x = imu.orientation.x
        self.magnetic_field_y = imu.orientation.y
        self.magnetic_field_z = imu.orientation.z
        self.IMU_input_Time = time.time()
        
    
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        if self.yaw  < 0:
            self.yaw  += 2*np.pi
 

        measurement =[[self.linear_x],[self.linear_y],[self.linear_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        # 업데이트 단계
        # updated_state = kf.update(measurement)
        gravity_vector = np.array([[0],[0],[9.80665]])
        

        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)
        # print(np.array(result))
        
        # self.linear_x2  = result[0][0]
        a = 8
        self.linear_x2 = int(result[0][0] * 10**a) / 10**a
        self.linear_y2 = int(result[1][0] * 10**a) / 10**a
        self.linear_z2 = int(result[2][0] * 10**a) / 10**a
        # print( self.linear_x2, self.linear_y2, self.linear_z2,self.IMU_vel_total )

        # self.linear_y2 = 0
        # print(self.linear_x2,self.linear_y2,self.yaw,self.IMU_vel_total)
        

    def IMU_HEADING(self):
        return self.yaw        
            
    # def IMU_velocity_calc(self):
    #     if self.IMU_init_Count == 0:
            
    #         self.IMU_last_accel_x = self.linear_x2
    #         self.IMU_last_accel_y = self.linear_y2
    #         self.IMU_last_Time  = time.time()
    #         self.IMU_init_Count = 1
        
            
    #     elif self.IMU_init_Count == 1:
    #         IMU_time_intervel = self.IMU_input_Time -self.IMU_last_Time 
    #         if IMU_time_intervel >= 0.001:  
    #             self.IMU_vel_x = (self.linear_x2+self.IMU_last_accel_x)*IMU_time_intervel/2 
    #             self.IMU_vel_y = (self.linear_y2+self.IMU_last_accel_y) * IMU_time_intervel/2 

    #             ###########################################################################
    #             if self.IMU_vel_total < 10:
    #                 self.timer = 0.1
    #                 self.IMU_vel_x = 0
    #                 self.IMU_vel_y = 0
    #             else:
    #                 self.timer = 0.5
    #             ###########################################################################    
    #             i = 1
    #             if self.IMU_vel_x > 0:
    #                 i = 1
    #             elif self.IMU_vel_x == 0:
    #                 i = 0
    #             else:
    #                 i = -1    
    #             self.IMU_vel_total += i*np.sqrt(self.IMU_vel_x**2+self.IMU_vel_y**2) * 3.6
    #             # self.IMU_vel_total += self.IMU_vel_x 
                
    #             if self.IMU_vel_total == 0:
    #                 self.vel_error = 0
    #             if self.IMU_vel_total<0:
    #                 self.IMU_vel_total = 0  
    #                 self.vel_error = 0

    #             # print(self.IMU_vel_total)

    #             self.IMU_last_Time = self.IMU_input_Time
    #             self.IMU_last_accel_x  = self.linear_x2
    #             self.IMU_last_accel_y  = self.linear_y2
             
    #     return self.IMU_vel_total
    

class save:
    def __init__(self):
        pass

    def save_data(self, file_path,array):
        # 자기장 데이터를 CSV 파일에 저장하는 함수
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'GPS', 'ENC','FILTER','FILTER_ORIGINAL'])
            writer.writerows(array)
     

def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    s = save()
    rate = rospy.Rate(10)

    X = []
    Y = []
    Z = []
    E = []
    F = []
    save_data = []

    meet_count = []
    meet_velocity = []

    sensor_data = []
    filtered_velocity = 0



    count = 0
    init_pos = [0,0]
    while init_pos == [0,0]:
        init_pos = p.GPS_POS()


    while not rospy.is_shutdown():  
        GPS_vel = p.GPS_VELOCITY()
        # IMU_Vel = p.IMU_velocity_calc()
        ERP_vel = p.erp_Velocity()
        count +=1

        if GPS_vel >= ERP_vel-1 and GPS_vel <= ERP_vel+1 :
            meet_count.append(count)
            meet_velocity.append(GPS_vel)
            print(count,GPS_vel)

        

        
        sensor_data.append(ERP_vel) # 실제 센서 데이터에 노이즈 추가

        # FFT 계산
        fs = 1000
        fft_result = np.fft.fft(sensor_data)
        freqs = np.fft.fftfreq(len(sensor_data), 1/fs)

        # 주파수 영역에서 노이즈 식별 및 필터링
        # 예제에서는 주파수가 50Hz의 성분을 제거하는 것으로 가정
        noise_freq_index = np.abs(freqs) > 50  # 50Hz 이상의 주파수 성분 인덱스
        fft_result[noise_freq_index] = 0  # 50Hz 이상의 주파수 성분을 0으로 설정 (노이즈 제거)

        # 역 FFT를 통한 시간 영역 데이터 복원
        filtered_signal = np.fft.ifft(fft_result)
        filtered_velocity = round(np.real(filtered_signal[-1]),1)
        print("filtered_",filtered_velocity)
        
        save_data.append([count,GPS_vel,ERP_vel,filtered_velocity,0])


        X.append(count)
        # Y.append(IMU_Vel)
        Z.append(GPS_vel) 
        E.append(ERP_vel)
        F.append(filtered_velocity)
        # if count > 100000000:
        #     break
        
        rate.sleep()
        
    

    start = 5    
    plt.figure(1)
    # plt.plot(X[start:], Y[start:], label='GPS+IMU VEL', color='blue')
    plt.plot(X[start:], Z[start:], label='GPS VEL', color='red')
    plt.plot(X[start:], E[start:], label='ENC VEL', color='green')
    plt.plot(meet_count[:], meet_velocity[:], label='meet_point', color='black')
    # plt.plot(X[start:], F[start:], label='Filtered Sensor Data', linestyle='--', linewidth=2,color='orange')
    plt.plot(X[start:], filtered_signal.real[start:], label='Filtered Sensor Data', linestyle='--', linewidth=2,color='orange')


    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')
    plt.legend()  

    plt.show()

    for i in range(len(filtered_signal)):
        save_data[i][4] = filtered_signal.real[i]

    s.save_data('velocity.csv',save_data)

if __name__ == "__main__":
    main()                           