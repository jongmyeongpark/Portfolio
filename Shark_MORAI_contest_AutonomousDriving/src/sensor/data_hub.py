#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import time
import os, sys
from math import pi
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion
from morai_msgs.msg import GPSMessage
from tf.transformations import euler_from_quaternion
from location import gps_imu_fusion
import numpy as np

from pyproj import Transformer, CRS

proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')

class SensorDataHub:
    def __init__(self):      
          
        self.message_buffer = []
        self.time_buffer = []
        
        # 구독자 선언
        self.sub_gps = rospy.Subscriber('/gps', GPSMessage, self.pose_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        # 발행자 선언
        self.localization_pub = rospy.Publisher('current_pose', Point, queue_size=1)
        self.speed_pub=rospy.Publisher('/speed', Float32, queue_size=1)

        # 사용하는 객체 선언
        self.loc_fusion = gps_imu_fusion()

        # flag 선언
        ##self.lidar_flag = False
        self.gps_flag = False
        self.imu_flag = False

        self.localization_flag = False

        self.previous_time = time.time()

        # Sub 받은 데이터 저장 공간
        self.sub_coord = [0.0, 0.0]
        self.sub_gps_heading = 0.0
        self.sub_imu_heading = 0.0
        self.obs_xyz = [0, 0, 0]
        self.sub_imu = Quaternion()
        self.linear_velocity = 0.0
        self.steer_angle = 0.0
        self.count = 0
        self.lidar_timestamp = None
        self.speed_rec_time = None
        self.pos2 = [0.0, 0.0]

        # obs_pub에 사용되는 msg 객체 선언
        self.obs_xyz = None
        
        
        self.pos = Point()
        self.local_pos = Point()
        self.wheelbase = 3  # [m]
        self.ego_speed = None
        self.yaw = 0
        self.pre_imu = 0
        self.pos1 = [0, 0]
        self.past_location = [0, 0]
        self.GPS_init_Count = 0
        self.countA = 0
        
        self.IMU_accel_x = 0
        
        self.GPS_last_Time  = time.time()
        self.GPS_input_Time = time.time()
        self.GPS_time_interval = 0
        self.GPS_velocity = 0
        self.GPS_last_velocity = 0
        self.edit_velocity = 0

    # #########callback 함수 모음##########
    # 각 센서에서 데이터가 들어오면 객체 내부의 데이터 저장공간에 저장
    def pose_callback(self, fix):
        time_stamp = fix.header.stamp.secs + fix.header.stamp.nsecs/1000000000

        self.sub_coord = [fix.longitude, fix.latitude, time_stamp]
        self.gps_flag = True

        ####################위도 X,Y좌표로 전환############## 0322 이승진 추가
        lon = fix.longitude
        lat = fix.latitude
        # x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        # print(x,y)
        self.pos1 = [x, y]
        self.GPS_input_Time = time.time()
        self.countA +=1
            

    def imu_callback(self, imu):
        # print("real_time:",time.time())
        #print("time_error:",time.time()-self.rec_time)

        # print("imu_time :",imu.header.stamp.secs)
        
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        if yaw < 0:
            yaw += 2*pi
        # print("--------------------")
        print("yaw : ",np.rad2deg(yaw))
        # print("pitch : ",pitch)
        self.sub_imu_heading = yaw
        self.imu_flag = True
        
        timestamp = imu.header.stamp
        time_1 = timestamp.secs + timestamp.nsecs * 1e-9
        
                ######0322 승진추가#########    
        
        IMU_raw_X = imu.linear_acceleration.x 
        IMU_raw_Y = imu.linear_acceleration.x 
        if (IMU_raw_X > 0.035 or IMU_raw_X < - 0.035) and (IMU_raw_Y > 0.035 or IMU_raw_Y < - 0.035):
            self.IMU_accel_x = IMU_raw_X * 9.8  # 가속도를 m/s^2로 변환
            self.IMU_accel_y = IMU_raw_Y * 9.8   
        
         
        # 버퍼에 데이터와 시간 저장
        self.message_buffer.append(yaw)
        self.time_buffer.append(time_1)

        while len(self.message_buffer) > 50:
            self.message_buffer.pop(0)
            self.time_buffer.pop(0)
            
        
    def GPS_velocity_calc(self):
        
        if self.GPS_init_Count == 0:
            gps_measurements_x =0
            gps_measurements_y =0
            for i in range(5):
                gps_measurements = self.pos1
                gps_measurements_x += gps_measurements[0]
                gps_measurements_y += gps_measurements[1]
                time.sleep(0.1)
            
            F_GPS_X = gps_measurements_x / 10
            F_GPS_Y = gps_measurements_y / 10
            
            self.GPS_position = [F_GPS_X,F_GPS_Y]
            self.GPS_last_Time = self.GPS_input_Time 
            self.GPS_init_Count = 1
           
            # print("setup")
            
        elif self.GPS_init_Count == 1:
            self.GPS_time_interval = self.GPS_input_Time - self.GPS_last_Time    
            if  self.GPS_time_interval >= 0.1:
                self.GPS_distance = np.hypot(self.pos1[0]-self.GPS_position[0],self.pos1[1]-self.GPS_position[1])
                self.GPS_velocity = self.GPS_distance / self.GPS_time_interval * 3.6

        
                # if abs(self.GPS_velocity - self.GPS_last_velocity) < self.GPS_last_velocity/2: #급격한 속도 감속(충돌)이 없을때
                if self.GPS_velocity > 3.0:
                    
                    error = 3.6* self.GPS_time_interval + 1
                
                    if self.IMU_accel_x >= 0 and self.GPS_velocity - self.GPS_last_velocity >= 0:
                        if (self.GPS_velocity > self.GPS_last_velocity + self.IMU_accel_x*self.GPS_time_interval + error):
                            self.GPS_velocity = self.GPS_last_velocity + self.IMU_accel_x*self.GPS_time_interval
                            # print("edit A")
                        else:
                            self.GPS_velocity = self.GPS_velocity
                        
                    elif self.IMU_accel_x <= 0 and self.GPS_velocity - self.GPS_last_velocity <= 0:
                        if (self.GPS_velocity < self.GPS_last_velocity + self.IMU_accel_x*self.GPS_time_interval - error):
                            self.GPS_velocity = self.GPS_last_velocity + self.IMU_accel_x*self.GPS_time_interval
                            # print("edit B")
                        else:
                            self.GPS_velocity = self.GPS_velocity
                            
                                                
                    else:
                        self.GPS_velocity = self.GPS_last_velocity
                        # print("edit C")
                        
                if(self.GPS_velocity<200):        
                    self.edit_velocity  = round(self.GPS_velocity,1)    
                    # print(self.edit_velocity,"일반 속도") 
                    
                self.GPS_last_Time = self.GPS_input_Time
                self.GPS_position = self.pos1 
                self.GPS_last_velocity = self.GPS_velocity
                
                if self.edit_velocity > 50:
                    self.edit_velocity = 50
                
                self.ego_speed = self.edit_velocity
    

    def localization_update(self, select_heading):
        if self.localization_flag is False:
            x, y = self.loc_fusion.tf_to_tm(self.sub_coord[0], self.sub_coord[1])
            self.pos.x = x
            self.pos.y = y
            self.pos.z = self.sub_imu_heading
            

            self.local_pos.x = self.pos.x
            self.local_pos.y = self.pos.y
            self.local_pos.z = self.pos.z
        else:
            self.pos.x = -1.0
            self.pos.y = 0
            self.pos.z = 0

    def pub_speed(self):
        if self.ego_speed == None:
            return
        
        self.speed_pub.publish(self.ego_speed)


    def pub_pose(self):
        self.localization_pub.publish(self.pos)

def main():
    # 기본 설정
    rospy.init_node('data_hub', anonymous=True)
    
    Data = SensorDataHub()
    rate = rospy.Rate(10)
    while Data.countA < 5:
        rate.sleep()

    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():

        # 각 정보 업데이트 후 발행
        Data.localization_update(2)
        Data.pub_pose()
            
        Data.GPS_velocity_calc()
            
        Data.pub_speed()
        # os.system('clear')
        # print ('Operating for reading sensor data')
        # print('Running Time: ', rospy.Time.now().to_sec() - start_time)
        rate.sleep()
        

if __name__ == '__main__':
    main()