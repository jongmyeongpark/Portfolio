import numpy as np
from pyquaternion import Quaternion
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
from pyproj import Transformer, CRS
import matplotlib.pyplot as plt
import time
from math import pi
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import CtrlCmd
import csv
import random
from std_msgs.msg import Float64,Float32
from geometry_msgs.msg import Point



proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')


ctrl_cmd_msg = CtrlCmd()


class Position:
    def __init__(self):
        # self.morai_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        # self.sub_gps = rospy.Subscriber("/gps", GPSMessage, self.gps_callback, queue_size=1)
        # self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
    
        # self.sub_steer = rospy.Subscriber('/steer_pub', Float32, self.steer_callback, queue_size=1)
        self.Dead_reckoning_pub=rospy.Publisher('/reckoning_pose', Point, queue_size=1) # x,y는 tm좌표, z에 들어가 있는 값이 heading
        self.Dead_reckoning_pos = Point()


        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.GPS_VELOCITY_COUNT = 0
        self.GPS_VELOCITY_dT = 0
        self.GPS_VELOCITY_VEL = 0 #속도
        self.GPS_VELOCITY_LAST_VEL = 0 #이전 속도
        self.GPS_VECOCITY_accel = 0
        self.GPS_VECOCITY_Heading = 0
        self.Distance = 0

        self.pos = [0,0]
        self.last_pos = [0,0]

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

        self.GPS_move_flag = 0
        self.input_steer = 0
        self.Dead_reckoning_init_count = 0
        self.count = 0
        self.signal_int = 0

    def steer_callback(self,data):
        self.input_steer = data.data    
        print("_________________________________",self.sub_steer)

    def gps_callback(self, gps_msg):
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()

    def pose_update(self, current_pose: Point):
        self.pos = [current_pose.x, current_pose.y]
        self.GPS_INPUT_TIME = time.time()

    def imu_callback(self,imu):
        self.IMU_RAW_accel_x = imu.linear_acceleration.x
        self.IMU_RAW_accel_y = imu.linear_acceleration.y
        self.IMU_RAW_accel_z = imu.linear_acceleration.z

        # print(imu.angular_velocity)

        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

        if self.yaw  < 0:
            self.yaw  += 2*np.pi

        measurement =[[self.IMU_RAW_accel_x],[self.IMU_RAW_accel_y],[self.IMU_RAW_accel_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.IMU_INPUT_TIME = time.time()

    def imu_update(self, imu: Imu):
        self.IMU_RAW_accel_x = imu.linear_acceleration.x
        self.IMU_RAW_accel_y = imu.linear_acceleration.y
        self.IMU_RAW_accel_z = imu.linear_acceleration.z

        # print(imu.angular_velocity)

        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

        if self.yaw  < 0:
            self.yaw  += 2*np.pi

        measurement =[[self.IMU_RAW_accel_x],[self.IMU_RAW_accel_y],[self.IMU_RAW_accel_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.IMU_INPUT_TIME = time.time()

    def update(self, current_pose, imu):
        self.pose_update(current_pose)
        self.imu_update(imu)

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
                self.Distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                if round(self.Distance,2) == 0:
                    self.GPS_move_flag = 0

                else: 
                    self.GPS_move_flag = 1        

                self.GPS_VECOCITY_Heading = np.arctan2(self.pos[1] - self.last_pos[1],self.pos[0] - self.last_pos[0])

                if self.GPS_VELOCITY_dT != 0:
                    self.GPS_VECOCITY_accel = (self.GPS_VELOCITY_LAST_VEL - self.GPS_VELOCITY_VEL) / self.GPS_VELOCITY_dT /3.6

                while self.GPS_VECOCITY_Heading > 2*pi:
                    self.GPS_VECOCITY_Heading -= 2*pi

                while self.GPS_VECOCITY_Heading < 0:
                    self.GPS_VECOCITY_Heading += 2*pi

                self.GPS_VELOCITY_VEL = self.Distance / self.GPS_VELOCITY_dT * 3.6

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.GPS_VELOCITY_LAST_VEL = self.GPS_VELOCITY_VEL

        return self.GPS_VELOCITY_VEL, self.GPS_VECOCITY_accel, self.GPS_VECOCITY_Heading, self.GPS_VELOCITY_dT,self.Distance

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

        return self.IMU_VELOCITY_VEL,self.IMU_ACCEL_X_avg,self.yaw,self.IMU_VELOCITY_dT
    
    def Dead_reckoning(self, current_pose, imu , v, steer, singal):

        self.update(current_pose, imu)

        if self.Dead_reckoning_init_count == 0:
            self.GPS_pos_x_plot = []
            self.GPS_pos_y_plot = []

            self.count = 0
            self.count_plot = []
            
            self.v_minus_count = 0
            steer_change = 0
            self.speed_cal = v
            v_plus_max = 0
            v_plus = 0

            INIT_POS = [0,0]
            self.GPS_POS_X_plot = []
            self.GPS_POS_Y_plot = []

            self.DEAD_RECKONING_START_POS = [0,0]
            self.DEAD_RECKONING_velocity_x = 0
            self.DEAD_RECKONING_velocity_y = 0

            self.LAST_DEAD_RECKONING_velocity_x = 0
            self.LAST_DEAD_RECKONING_velocity_y = 0

            DEAD_RECKONING_move_x = 0
            DEAD_RECKONING_move_y = 0

            DEAD_RECKONING_X = 0
            DEAD_RECKONING_Y = 0

            self.DEAD_RECKONING_X_plot = []
            self.DEAD_RECKONING_Y_plot = []

            self.last_steer = 0
            self.IMU_heading_EDIT = 0
            self.IMU_heading_EDIT_array = []

            self.IMU_heading_array = []
            self.GPS_heading_array = []
            self.signal = 0

            self.Dead_reckoning_init_count = 1

        elif self.Dead_reckoning_init_count == 1:   
            GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt,GPS_distance = self.GPS_VELOCITY()
            IMU_velocity,IMU_accel,IMU_heading,IMU_dt = self.IMU_VELOCITY()

            if singal != 2:
                self.signal = singal

            else:
                if self.signal_int == 0:
                    self.signal = singal

            if steer == 0:
                steer_change = 0
            else:    
                steer_change = 1

            if self.last_steer * steer > 0:
                steer_change = 1
            else:
                steer_change = 0

            if steer_change == 1:
                if abs(steer) >= 1:
                    v_plus_max = abs(abs(0.15*v) - 0.5)
                else:
                    v_plus_max = abs(steer/2)-0.05  
                v_time = (-0.025*abs(steer)+0.4)*v + (0.35*abs(steer)-4)+4
                v_plus = v_plus_max / v_time
                self.speed_cal = self.speed_cal + v_plus
                print("A",steer,self.last_steer)

            elif steer_change == 0:
                if abs(steer) >= 1:
                    v_plus_max = abs(abs(0.15*v) - 0.5) #v_plus_max = abs(abs(0.15*v) - 0.5)
                else:
                    v_plus_max = abs(steer/2)-0.05
                v_time = (-0.025*abs(steer)+0.4)*v + (0.35*abs(steer)-4)+4
                v_plus = v_plus_max / v_time
                self.speed_cal = self.speed_cal - v_plus    
                print("B",steer,self.last_steer)
                self.v_minus_count += 1
            
            if self.v_minus_count == 0 or self.v_minus_count == 5:
                self.last_steer = steer
                self.v_minus_count = 0

            if self.speed_cal > v + v_plus_max:
                self.speed_cal = v + v_plus_max

            if self.speed_cal < v:
                self.speed_cal = v

            self.speed_cal = int(self.speed_cal*10)/10
            print(self.speed_cal,IMU_heading,"/",GPS_pos_velocity,GPS_pos_heading)
            # process_timer = abs(check_time - time.time())
            # process_timer = 0.1
            process_timer = GPS_pos_dt


            self.count += 1
            self.count_plot.append(self.count)
            # self.GPS_pos_x_plot.append(gps_pos[0])
            # self.GPS_pos_y_plot.append(gps_pos[1])

            # DEAD_RECKONING_move_x = (DEAD_RECKONING_velocity_x) * process_timer
            # DEAD_RECKONING_move_y = (DEAD_RECKONING_velocity_y) * process_timer
            
            self.LAST_DEAD_RECKONING_velocity_x = self.DEAD_RECKONING_velocity_x
            self.LAST_DEAD_RECKONING_velocity_y = self.DEAD_RECKONING_velocity_y

            DEAD_RECKONING_START = 100
            if self.signal == 1:
                D_heading = 0
                if self.GPS_move_flag == 1:
                    if GPS_pos_heading >= IMU_heading:
                        D_heading = GPS_pos_heading - IMU_heading
                        print("______A",D_heading)
                    else:
                        D_heading = IMU_heading - GPS_pos_heading 
                        print("______B",D_heading)

                    self.IMU_heading_EDIT_array.append(D_heading)

            if self.signal == 2:
                self.IMU_heading_EDIT = np.mean(self.IMU_heading_EDIT_array)

                INIT_POS = self.GPS_POS()
                self.DEAD_RECKONING_START_POS = INIT_POS

                self.GPS_POS_X_plot.append(INIT_POS[0])
                self.GPS_POS_Y_plot.append(INIT_POS[1])
                self.DEAD_RECKONING_X_plot.append(self.DEAD_RECKONING_START_POS[0])
                self.DEAD_RECKONING_Y_plot.append(self.DEAD_RECKONING_START_POS[1])

                # percent = 0.87 # 속도 20일때
                # percent = 0.71 # 속도 10일때
                # self.percent1 = 0.59
                # self.percent2 = 0.555

                self.percent1 = 0.5816
                # self.percent2 = 0.5486                self.mission_reckoning.Dead_reckoning(v=self.ctrl_speed, steer=self.ctrl_steer, singal=1)

                self.percent2 = 0.549

                # self.percent1 = 0.6
                # self.percent2 = 0.555

                # self.percent1 = 0.6
                # self.percent2 = 0.561
                self.signal = 3

            if self.signal == 3:
                self.signal_int = 1
                IMU_heading = IMU_heading - self.IMU_heading_EDIT
                print("DEAD_RECKONING . . .",self.IMU_heading_EDIT,IMU_heading)

                self.DEAD_RECKONING_velocity_x = np.cos(IMU_heading) * self.speed_cal / 3.6 #km/h => m/s
                self.DEAD_RECKONING_velocity_y = np.sin(IMU_heading) * self.speed_cal / 3.6

                DEAD_RECKONING_move_x = ((self.DEAD_RECKONING_velocity_x + self.LAST_DEAD_RECKONING_velocity_x)/2) * process_timer
                DEAD_RECKONING_move_y = ((self.DEAD_RECKONING_velocity_y + self.LAST_DEAD_RECKONING_velocity_y)/2) * process_timer

                # DEAD_RECKONING_move_x = (self.DEAD_RECKONING_velocity_x) * process_timer
                # DEAD_RECKONING_move_y = (self.DEAD_RECKONING_velocity_y) * process_timer

                # print("-----------------------",DEAD_RECKONING_move_x,DEAD_RECKONING_move_y)
                self.DEAD_RECKONING_START_POS[0] += DEAD_RECKONING_move_x * self.percent1
                self.DEAD_RECKONING_START_POS[1] += DEAD_RECKONING_move_y * self.percent2 

                DEAD_RECKONING_X = self.DEAD_RECKONING_START_POS[0]
                DEAD_RECKONING_Y = self.DEAD_RECKONING_START_POS[1]

                self.Dead_reckoning_pos.x = DEAD_RECKONING_X
                self.Dead_reckoning_pos.y = DEAD_RECKONING_Y
                self.Dead_reckoning_pos.z = IMU_heading
                self.Dead_reckoning_pub.publish(self.Dead_reckoning_pos)

                GPS_POS = self.GPS_POS()
                self.GPS_POS_X_plot.append(GPS_POS[0])
                self.GPS_POS_Y_plot.append(GPS_POS[1])

                self.DEAD_RECKONING_X_plot.append(DEAD_RECKONING_X)
                self.DEAD_RECKONING_Y_plot.append(DEAD_RECKONING_Y)

                print("-------------------------X",GPS_POS[0]," PREDICT_POS_X ",DEAD_RECKONING_X,"-----------Y",GPS_POS[1]," PREDICT_POS_Y ",DEAD_RECKONING_Y)

                self.LAST_DEAD_RECKONING_velocity_x = self.DEAD_RECKONING_velocity_x
                self.LAST_DEAD_RECKONING_velocity_y = self.DEAD_RECKONING_velocity_y

            self.IMU_heading_array.append(IMU_heading)
            self.GPS_heading_array.append(GPS_pos_heading)

            if self.signal == 4:
                plt.figure(1)
                plt.scatter(self.GPS_POS_X_plot[0], self.GPS_POS_Y_plot[0], label='GPS', color='red')
                plt.scatter(self.DEAD_RECKONING_X_plot[0], self.DEAD_RECKONING_Y_plot[0], label='DEAD_RECKONING', color='blue')
                plt.plot(self.GPS_POS_X_plot[:], self.GPS_POS_Y_plot[:], label='GPS', color='red')
                plt.plot(self.DEAD_RECKONING_X_plot[:], self.DEAD_RECKONING_Y_plot[:], label='DEAD_RECKONING', color='blue')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('position')

                plt.figure(2)
                plt.scatter(self.GPS_POS_X_plot[0], self.GPS_POS_Y_plot[0], label='GPS', color='red')
                plt.plot(self.GPS_POS_X_plot[:], self.GPS_POS_Y_plot[:], label='GPS', color='red')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('position')

                plt.figure(3)
                plt.scatter(self.DEAD_RECKONING_X_plot[0], self.DEAD_RECKONING_Y_plot[0], label='DEAD_RECKONING', color='blue')
                plt.plot(self.DEAD_RECKONING_X_plot[:], self.DEAD_RECKONING_Y_plot[:], label='DEAD_RECKONING', color='blue')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('position')

                plt.figure(4)
                plt.plot(self.count_plot[:], self.GPS_heading_array[:], label='DEAD_RECKONING', color='red')
                plt.plot(self.count_plot[:], self.IMU_heading_array[:], label='DEAD_RECKONING', color='blue')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('position')

                plt.show()

            if self.signal == 0:
                self.Dead_reckoning_init_count = 0
            

def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    rate = rospy.Rate(10)
    v = 7
    check_time = time.time()
    while not rospy.is_shutdown():
        dt = abs(time.time() - check_time)
            
        steer = p.input_steer

        ctrl_cmd_msg.longlCmdType = 2
        ctrl_cmd_msg.brake = 0
        ctrl_cmd_msg.velocity = v
        ctrl_cmd_msg.steering = steer
        p.morai_pub.publish(ctrl_cmd_msg) 

        if dt > 5 and dt < 10:
            p.Dead_reckoning(v,steer,1) ##헤딩값 보정 (음영구간)

        elif dt >= 10 and dt < 70:
            p.Dead_reckoning(v,steer,2) ##데드 렉코닝 시작

        elif dt >= 70:
            p.Dead_reckoning(v,steer,4) ##그래프 출력 // ##0은 데드렉코닝 초기화


        rate.sleep()

    ctrl_cmd_msg.longlCmdType = 2
    ctrl_cmd_msg.brake = 1
    ctrl_cmd_msg.velocity = v
    p.morai_pub.publish(ctrl_cmd_msg) 

if __name__ == "__main__":
    main()

