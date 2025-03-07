#!/usr/bin/env python3
#-*-coding:utf-8-*-


# 라이브러리 임포트
import os, sys
import time
import rospy
import os, sys
import numpy as np
from math import cos, sin , sqrt

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/missions")


# message 파일
import rospy
import numpy as np
from geometry_msgs.msg import Point, Point32 , Vector3
from std_msgs.msg import Header, ColorRGBA, Bool 
import threading
from math import *
from sensor.find_parking_area import *
import path_planning.cubic_spline_planner as cubic_spline_planner
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from sensor.parking import ParkingDetector


## Parameters
WB = 1.04 # wheelbase


WIDTH= 1.16
LENGTH= 2.02 

rear2bumper=0.5

STOP_TIME = 5


class mission_parking:
    def __init__(self,PARKING_LEN,PARKING_WIDTH,goal_point,parking_heading):


        self.ANGLE = np.radians(23) 
        self.R_turn=WB/np.tan(self.ANGLE)

        self.parking_status=0
        self.feature_points=[]
        self.theta_3=0.0
        self.map_generated=[]
        self.map_generated2=[]
        self.start_pose=[]

        self.forfront = []
        self.forback  = []
        self.front_waypoint = []
        self.back_waypoint = []

        self.goal_point = goal_point
        self.parking_heading = parking_heading

        self.PARKING_LEN= PARKING_LEN
        self.PARKING_WIDTH= PARKING_WIDTH

        self.map_thread=None

        self.detector = ParkingDetector()

        self.detection_flag = False
        self.search_step = 0       
        self.parking_path = None
        self.reverse_check = 0

        self.check_time = 0.0

        self.counted = 0
        self.check_flag = False

        self.arrangement_front_check = False
        self.arrangement_back_check = False

        self.arrangement_front_first = True
        self.arrangement_back_first = True


        #Publisher
        self.parking_pub = rospy.Publisher('/parking_delivery', PointCloud, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)
        self.vizpath_pub = rospy.Publisher('/parking_vizpath', Marker, queue_size=1)


    def find_point(self,cur_pose):
        
        self.start_pose=[cur_pose.x,cur_pose.y]

        x1= 1.0
        y1= 0.8

        x1_2= -1.5
        y1_2= -0.6
        

        theta_a=np.arcsin((self.PARKING_LEN -(rear2bumper+0.3) )/np.sqrt((self.PARKING_LEN - (rear2bumper +0.3) )**2+( self.R_turn - (self.PARKING_WIDTH/2))**2))
        
        theta_b=np.arccos((self.R_turn + WIDTH/2 + 0.7)/np.sqrt((self.PARKING_LEN- (rear2bumper + 0.3))**2+( self.R_turn - (self.PARKING_WIDTH/2))**2))
        
        theta_3=theta_a - theta_b
        
        x2= x1 + self.R_turn*np.sin(theta_3) 
        x2_2= x1_2 + self.R_turn*np.sin(theta_3)
        
        y2= y1 + self.R_turn*(1-np.cos(theta_3))

        y2_2 =y1_2 + self.R_turn*(1-np.cos(theta_3))
        
        x3= x2 + (self.R_turn + WIDTH/2)*np.tan(theta_b)*np.cos(theta_3)
        x3_2= x2_2 + (self.R_turn + WIDTH/2)*np.tan(theta_b)*np.cos(theta_3)

        y3= y2 + (self.R_turn + WIDTH/2)*np.tan(theta_b)*np.sin(theta_3)
        y3_2=y2_2 + (self.R_turn + WIDTH/2)*np.tan(theta_b)*np.sin(theta_3)        
        
        x4= x3 + self.R_turn*np.sin(theta_3) 
        
        x4_2= x3_2 + self.R_turn*np.sin(theta_3) 

        
        y4= y3 + self.R_turn*(1-np.cos(theta_3))
        
        y4_2= y3_2 + self.R_turn*(1-np.cos(theta_3))

        
        x5 = x4 + 1.0
        
        x5_2 = x4_2 + 2.0
        
        y5 = y4 
        
        y5_2 = y4_2

        
        x6 = x1 - 5
        
        x6_2 = x1_2 -1
        
        y6 = y1
        
        y6_2 = y1_2

        
        H_min = y5 - WIDTH/2 - self.PARKING_WIDTH

        forfront = [1.5 , 0]
        forback =  [-2.0 ,0]

        self.feature_points= [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x5,y5],[x6,y6]]
        self.feature_points2= [[x1_2,y1_2],[x2_2,y2_2],[x3_2,y3_2],[x4_2,y4_2],[x5_2,y5_2],[x6_2,y6_2]]
        self.theta_3=theta_3

        self.forfront = forfront
        self.forback = forback
        
        print("proceed to status 1")
        print(f"H_min : {H_min}")
        
        self.parking_status=1

    def move2start(self,cur_pose): 
        
        global_waypoints = self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],[self.feature_points[4]])
        
        global_waypoints = np.vstack((np.array(self.start_pose),global_waypoints))

        waypoints = self.spline_interpolation(global_waypoints)
        
        waypoints = np.vstack((waypoints,np.array([0, 0, 1])))
        
        print("Path for start point")
    
        self.visualize_path(cur_pose,waypoints)
        self.pub_parking_path(waypoints)
        
        distance = sqrt((cur_pose.x-waypoints[-2][0])**2 +(cur_pose.y-waypoints[-2][1])**2)
        
        print(f'dis1: {distance}')
        
        if distance < 1.0 :
            print("proceed to status 2")
            self.generate_map_async()
            self.stop_pub.publish(True)

            self.parking_status = 2

            self.check_time = rospy.Time.now().to_sec()

    
    def start2backpoint(self, cur_pose ):
        
        if self.map_generated != []:
            
            
            self.visualize_path(cur_pose, self.map_generated)
            
            self.pub_parking_path(self.map_generated)

            distance = sqrt((cur_pose.x-self.map_generated[-2][0])**2 +(cur_pose.y- self.map_generated[-2][1])**2)
            heading_diff = abs(self.parking_heading[self.search_step] - cur_pose.z)


            print("****")        
            print(f'dis2: {distance}')
            print(f'heading_diff : {heading_diff}')
            
            if (distance < 5 and heading_diff <= np.deg2rad(15)) or distance < 4.7:

                self.stop_pub.publish(True)
                self.parking_status = 3
        else:
            
            print("stop")
            print("======")
            print("waiting for map......")
            self.stop_pub.publish(True)
            

    def generate_map(self): 

            
        arc2=self.point_division(self.feature_points[2],self.feature_points[3],self.theta_3,mode=2)
        line=self.point_division(self.feature_points[1],self.feature_points[2],self.theta_3,mode=1)
        arc1=self.point_division(self.feature_points[0],self.feature_points[1],self.theta_3,mode=0)
        
        
        arc1_points=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],arc1)
        line_points=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],line)
        arc2_points=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],arc2)
        
        line2_points=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],[self.feature_points[3],self.feature_points[4]])
        line3_points=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],[self.feature_points[5],self.feature_points[0]])
        
        zero = np.vstack((line3_points,arc1_points[1:]))
        first=np.vstack((zero,line_points[1:]))
        second=np.vstack((first,arc2_points[1:]))
        total_points=np.vstack((second,line2_points[1:]))
        
        total_waypoint = self.spline_interpolation(total_points)
        
        total_waypoint = total_waypoint[::-1]
        
        total_waypoint = np.vstack((total_waypoint,np.array([0, 0 , -1])))
        arc2_2=self.point_division(self.feature_points2[2],self.feature_points2[3],self.theta_3,mode=2)
        line_2=self.point_division(self.feature_points2[1],self.feature_points2[2],self.theta_3,mode=1)
        arc1_2=self.point_division(self.feature_points2[0],self.feature_points2[1],self.theta_3,mode=0)
        
        arc1_points2=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],arc1_2)
        line_points2=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],line_2)
        arc2_points2=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],arc2_2)
        
        line2_points2=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],[self.feature_points2[3],self.feature_points2[4]])
        line3_points2=self.coord_conversion(self.goal_point[self.search_step],self.parking_heading[self.search_step],[self.feature_points2[5],self.feature_points2[0]])
        
        zero2 = np.vstack((line3_points2,arc1_points2[1:]))
        first2 =np.vstack((zero2,line_points2[1:]))
        second2 =np.vstack((first2,arc2_points2[1:]))
        total_points2 =np.vstack((second2,line2_points2[1:]))
    
        total_waypoint2 = self.spline_interpolation(total_points2)
        
        total_waypoint2 = total_waypoint2[::-1]
        
        total_waypoint2 = np.vstack((total_waypoint2,np.array([0, 0 , -1])))    
        
        self.map_generated = total_waypoint
        self.map_generated2 = total_waypoint2

            
    def generate_map_async(self ):

        map_thread = threading.Thread(target=self.generate_map())
        map_thread.start()


    def change_parking_status(self):
        self.parking_status +=1

    def reverse_parking_path(self):
        self.map_generated2 = self.map_generated2[:-1]

        self.map_generated2 = self.map_generated2[::-1]

        self.map_generated2 = np.vstack((self.map_generated2,np.array([0 , 0 , 1])))
        
    
    def back2glob(self,current_pose):
        if self.reverse_check==0:
            self.reverse_parking_path()
            self.reverse_check +=1
       
        self.visualize_path(current_pose, self.map_generated2)
            
        self.pub_parking_path(self.map_generated2)

        dist = sqrt((current_pose.x - self.map_generated2[-2][0])**2 + (current_pose.y - self.map_generated2[-2][1])**2)

        print(f'dist : {dist}')

        if dist <= 1.5 :
            self.change_parking_status()

    def arrangement_front(self,current_pose):
        if self.arrangement_front_first:
            self.arrangement_front_generate(current_pose)
            self.arrangement_front_first = False

        else:

            self.visualize_path(current_pose, self.front_waypoint)
                
            self.pub_parking_path(self.front_waypoint)

            dist = sqrt((current_pose.x - self.front_waypoint[-2][0])**2 + (current_pose.y - self.front_waypoint[-2][1])**2)
            
            print(f'arrange_front_dist : {dist}')

            if dist < 0.5:
                self.arrangement_front_check = True

        
    def arrangement_front_generate(self , current_pose):
        cur_pos = [current_pose.x , current_pose.y]

        first_point = self.coord_conversion(cur_pos, self.parking_heading[self.search_step] ,[self.forfront])

        first_point = np.vstack((first_point , cur_pos))

        first_point = first_point[::-1]

        self.front_waypoint = self.spline_interpolation(first_point)
        
        

        self.front_waypoint= np.vstack((self.front_waypoint,np.array([0, 0 , 1])))

    def arrangement_back(self,current_pose):
        if self.arrangement_back_first:
            self.arrangement_back_generate(current_pose)
            self.arrangement_back_first = False
        
        else:

            self.visualize_path(current_pose, self.back_waypoint)
                
            self.pub_parking_path(self.back_waypoint)

            dist = sqrt((current_pose.x - self.back_waypoint[-2][0])**2 + (current_pose.y - self.back_waypoint[-2][1])**2)

            print(f'arrange_back_dist : {dist}')

            if dist < 0.8:
                self.arrangement_back_check = True

    def arrangement_back_generate(self, current_pose):
        cur_pos = [current_pose.x , current_pose.y]
        first_point = self.coord_conversion(cur_pos, self.parking_heading[self.search_step] ,[self.forback])

        first_point = np.vstack((first_point, cur_pos))

        self.back_waypoint = self.spline_interpolation(first_point)

        self.back_waypoint = self.back_waypoint[::-1]
        
        self.back_waypoint= np.vstack((self.back_waypoint,np.array([0, 0 , -1])))

    def point_division(self,start_point,end_point,angle_diff,mode):
        
        points=[start_point]
        
        if mode==0: #arc1
            angle=angle_diff/3
            for index in range(1,3,1):
                
                points.append([start_point[0] + self.R_turn * np.sin(angle*index) , 
                               start_point[1] + (self.R_turn-self.R_turn*np.cos(angle*index))])

            points.append(end_point)
            
            return points
        
        elif mode==1: #line
            
            points.append(end_point)
            
            return points
            
        elif mode==2: #arc2
            angle = angle_diff/3
            for index in range(2,0,-1):

                points.append([end_point[0] -  (self.R_turn + 0.3) * np.sin(angle*index), end_point[1] - (self.R_turn + 0.3- (self.R_turn +0.3) * np.cos(angle*index)) ])
                
            points.append(end_point)
            return points 
        
        
    
    def coord_conversion(self,global_point, cur_heading,feature_points):
        # obs_tm을 빈 2D 배열로 초기화
        obs_tm = np.empty((0, 2))  

        # 회전 행렬 (z축에 대한 회전)
        T = np.array([
            [cos(cur_heading), -sin(cur_heading), 0],
            [sin(cur_heading),  cos(cur_heading), 0],
            [0,                    0,             1]
        ])

        # 이동 행렬
        T2 = np.array([
            [1, 0, global_point[0]],
            [0, 1, global_point[1]],
            [0, 0, 1]
        ])
        
        B = np.array([
            [1 , 0 , 0],
            [0 , 1 , 0],
            [0 , 0 , 1]
        ]) 
        

        for point in feature_points:
            # 입력 좌표를 동차 좌표로 변환
            homogeneous_point = np.array([point[0], point[1], 1])
            
            # 회전 및 이동 변환 적용
            transformed_point = np.dot(T2, B)
            transformed_point = np.dot(transformed_point,T)
            transformed_point = np.dot(transformed_point, homogeneous_point)
            
            # 변환된 좌표를 obs_tm에 추가 (x, y만 저장)
            obs_tm = np.vstack((obs_tm, transformed_point[:2]))  # [x, y]만 저장

        return obs_tm
 
    # parking path for MPC
    def pub_parking_path(self, waypoints):
        self.parking_path = PointCloud()
        for point in waypoints:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            self.parking_path.points.append(p)
        self.parking_pub.publish(self.parking_path)


    # parking path for rviz
    def visualize_path(self, current_pose, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0.0,b=1.0,a=0.8)
        )
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0] - current_pose.x
            p.y = waypoint[1] - current_pose.y
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.vizpath_pub.publish(rviz_msg_path)
 
    # cubic_spline
    def spline_interpolation(self,points):
        waypoint=np.empty((0,3))
        
        path_x = [point[0] for point in points]
        path_y = [point[1] for point in points]
        
        cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)
        for i in range(len(cx)):
            
            point=[cx[i],cy[i],cyaw[i]]           
            waypoint = np.vstack((waypoint, np.array(point)))
            
            
        return waypoint
    

    def step_check(self,current_pose):
        min_dis=10000
        check_ind=5
        for ind, point in enumerate(self.goal_point):
            dis = sqrt((point[0] - current_pose.x)**2 +(point[1] - current_pose.y)**2 )
            if dis < min_dis:
                min_dis = dis
                check_ind=ind
                
        return check_ind
    
    def pub_endsignal(self):
        end_path = np.array([0 , 0 , 0])
        end_path = np.vstack((end_path , np.array([0 , 0 , 0])))

        self.pub_parking_path(end_path)
    
    def run(self,current_pose):

        if not self.detection_flag:

            self.search_step=self.step_check(current_pose)
            
            print(f"Now Searching Parking{self.search_step + 1}....")

            if not self.check_flag:
                
                is_finded = self.detector.is_finded

                if is_finded: 
                    print("detected")
                    
                    self.check_flag = True
                   
                    self.counted += 1

                else:
                    pass
    
            else:
                is_finded = self.detector.is_finded

                if is_finded and self.counted < 3:
                    print("detected")
                    self.counted += 1

                elif is_finded and self.counted >= 3:
                    print("detected")
                    self.detection_flag = True

                    print(f"parking {self.search_step +1} is possible")
                    

                else:
                    self.counted = 0
                    self.check_flag = False

            print("============================")
                    
        else:            
            
            if self.parking_status == 0:
                self.find_point(current_pose)
                
            elif self.parking_status == 1:
                self.move2start(current_pose)
                
            elif self.parking_status == 2:
                time_passed = rospy.Time.now().to_sec() - self.check_time

                print(f"time_passed : {time_passed}")

                if time_passed >= 1:
                    self.start2backpoint(current_pose)

                else:
                    self.stop_pub.publish(True)
                    
            elif self.parking_status == 3: #전진 짧게
                self.arrangement_front(current_pose)

                if self.arrangement_front_check:
                    self.change_parking_status()

            elif self.parking_status == 4: # 후진 짧게
                self.arrangement_back(current_pose)

                if self.arrangement_back_check:

                    print(f"STOP!!!! for {STOP_TIME}seconds")
                    
                    self.change_parking_status()

                    self.check_time = rospy.Time.now().to_sec()

            elif self.parking_status == 5:
                time_passed = rospy.Time.now().to_sec() - self.check_time

                print(f"time_passed : {time_passed}")

                self.stop_pub.publish(True)

                if time_passed >= STOP_TIME:
                    self.change_parking_status()
                    print("Back to global path")
            
            elif self.parking_status == 6:
                self.back2glob(current_pose)

            else:
                print("Mission Parking comeplete!!!")
                self.pub_endsignal()

class path_generator:
    def __init__(self,PARKING_LEN,PARKING_WIDTH,local_coord,goal_point,goal_point2,goal_global,moment_pose,parking_heading,ratio):

        self.ANGLE = np.radians(25) 
        self.R_turn=WB/np.tan(self.ANGLE)

        self.delivery_status=0
        self.feature_points=[]

        self.theta_3=0.0
        
        self.map_generated=[]
        self.map_generated2=[]
        self.back_waypoint = []

        self.forback = []

        self.start_pose=[]

        self.goal_point = goal_point
        self.goal_point2 = goal_point2
        self.parking_heading = parking_heading
        self.goal_global = goal_global

        self.PARKING_LEN= PARKING_LEN
        self.PARKING_WIDTH= PARKING_WIDTH

        self.ratio = ratio

        self.map_thread=None
    
        self.delivery_path = None
        self.end_path = None
        self.reverse_check = 0
        self.back_check = 0
        
        self.local_coord = local_coord
        self.moment_pose = moment_pose

        self.current_pose_x = None
        self.current_pose_y = None
        self.heading = None

        self.current_pose = None


        self.import_module = mission_parking(PARKING_LEN,PARKING_WIDTH,goal_point,parking_heading)

        #Publisher
        self.delivery_pub = rospy.Publisher('/parking_delivery', PointCloud, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)
        self.vizpath_pub = rospy.Publisher('/delivery_vizpath', Marker, queue_size=1)
    


    def find_point(self,cur_pose):

        x6_3 = -3.0
        y6_2 = 0.0
        

        self.forback = [x6_3 , y6_2]


        
        self.delivery_status=1

    def B_delivery_getin_start(self,cur_pose):

        print("generate map for delivery B")

        self.generate_map_async_delivery(cur_pose)

        self.delivery_status = 2
    
    def start2point(self, cur_pose):
        
        if self.map_generated != []:
            
            self.visualize_delivery_path(cur_pose, self.map_generated)
            
            self.pub_delivery_path(self.map_generated)

            distance = sqrt((cur_pose.x-self.map_generated[-2][0])**2 +(cur_pose.y- self.map_generated[-2][1])**2)
            heading_diff = abs(self.parking_heading[0] - cur_pose.z)

            print("****")        
            print(f'dis2: {distance}')
            print(f'heading_diff : {heading_diff}')

            print(f"local : {self.local_coord, self.goal_point, self.goal_point2}")
            
            if (distance < 3 and heading_diff <= np.deg2rad(15)) or distance < 1.8:

                self.stop_pub.publish(True)
                self.delivery_status = 3
        else:
            
            print("stop")
            print("======")
            print("waiting for map......")
            self.stop_pub.publish(True)
        
    def coord_revision(self,points):
        revised =[]
        for point in points :
            x , y = point
            new_x = x - 3.0
            new_y = y 
            # if -0.5 <= self.ratio <= 0:
            #     new_y = y
            # else:
            #     new_y = y + 0.012 * (self.ratio)
            point = [new_x , new_y]
            revised.append(point)
        return revised
    
    def transform_local2global(self,points) -> np.ndarray:
        ''' convert local coord to global coord
        Args:
            local_points: lidar local points
        
        Return:
            global_points: lidar+gps -> global points
        '''

        if self.moment_pose.z is not None and self.moment_pose.x is not None and self.moment_pose.y is not None:
            T = [[np.cos(self.moment_pose.z), -np.sin(self.moment_pose.z), self.moment_pose.x],
                 [np.sin(self.moment_pose.z), np.cos(self.moment_pose.z), self.moment_pose.y],
                 [0, 0, 1]]
            
            global_points = []
            for (obs_x, obs_y) in points:
                obs_tm = np.dot(T, np.transpose([obs_x+1.04, obs_y, 1]))
                gx = obs_tm[0]
                gy = obs_tm[1]
                global_points.append([gx, gy])

            return global_points
        return []
    
    def update_current_pose(self, current_pose: Point):
        '''get current_pose [global_x, global_y, heading]
        '''
        self.current_pose_x = current_pose.x
        self.current_pose_y = current_pose.y
        self.heading = current_pose.z
        
        ###
        self.current_pose = current_pose
            

    def generate_map_delivery(self,cur_pose): 

        # arc2=self.point_division(self.feature_points2[2],self.feature_points2[3],self.theta_3,mode=2)
        # arc2_reversed = self.reflect_x(arc2)
        # arc2_revised = self.coord_revision(arc2_reversed)
        
        # line=self.point_division(self.feature_points2[1],self.feature_points2[2],self.theta_3,mode=1)
        # line_reversed = self.reflect_x(line)
        # line_revised = self.coord_revision(line_reversed)

        # arc1=self.point_division(self.feature_points2[0],self.feature_points2[1],self.theta_3,mode=0)
        # arc1_reversed = self.reflect_x(arc1)
        # arc1_revised = self.coord_revision(arc1_reversed)

        # arc1_points=self.import_module.coord_conversion(self.goal_point,self.parking_heading[0],arc1_revised)
        # line_points=self.import_module.coord_conversion(self.goal_point,self.parking_heading[0],line_revised)
        # arc2_points=self.import_module.coord_conversion(self.goal_point,self.parking_heading[0],arc2_revised)
        
        # line2_reversed = self.reflect_x([self.feature_points2[3],self.feature_points2[4]])
        # line2_revised = self.coord_revision(line2_reversed)
        # line2_points=self.import_module.coord_conversion(self.goal_point,self.parking_heading[0],line2_revised)

        # line3_reversed = self.reflect_x([self.feature_points2[5],self.feature_points2[0]])
        # line3_revised = self.coord_revision(line3_reversed)
        # line3_points=self.import_module.coord_conversion(self.goal_point,self.parking_heading[0],line3_revised)
        
        # zero = np.vstack((line3_points,arc1_points[1:]))
        # first=np.vstack((zero,line_points[1:]))
        # second=np.vstack((first,arc2_points[1:]))
        # total_points=np.vstack((second,line2_points[1:])) 

        # total_points = total_points[::-1]

        # total_waypoint = self.import_module.spline_interpolation(total_points)
        # point_array = [[0.0 , 0.0] ,[2.0,0.0],[self.goal_point[0] - 1.0 , 0] ,self.goal_point, self.goal_point2,self.local_coord[0]]
        # point_array = [[0.0 , 0.0] ,[2.0,0.0],[self.goal_point[0]-4 , 0], self.goal_point ,self.local_coord[0]]
        # second_point_ = [0,0]
        # if self.goal_global[0]-6 < 2:
        #     second_point_ = [2,0]
        # else:   
        #     second_point_ = [self.goal_point[0]-6 , 0]
        
        # point_array = [[0,0] ,second_point_, [self.goal_point[0]-4 , 0], self.goal_point ,self.local_coord[0]]
        # point_array = [[0,0] ,second_point_, self.goal_point, self.goal_point2 ,self.local_coord[0]]
        point_array = [[0,0],[2.5,0] , self.goal_point, self.goal_point2 ,self.local_coord[0]]
        # point_array = [[0,0] ,second_point_, [self.goal_point[0], self.goal_point[0]+0.5], [self.goal_point2[0], self.goal_point2[1]+0.5] ,[self.local_coord[0][0], self.local_coord[0][1]+0.5]]

        global_point = self.transform_local2global(point_array)
        
        # moment_pose = np.array([self.moment_pose.x , self.moment_pose.y])        
        # total_waypoint = np.vstack((moment_pose , global_point))
        total_waypoint = self.import_module.spline_interpolation(global_point)
    

        total_waypoint = np.vstack((total_waypoint,np.array([0, 0 , 1])))


        
        self.map_generated = total_waypoint
     
            
    def generate_map_async_delivery(self,cur_pose):

        map_thread = threading.Thread(target=self.generate_map_delivery(cur_pose))
        map_thread.start()


    def change_delivery_status(self):
        self.delivery_status +=1

    def reverse_delivery_path(self):
        self.map_generated2 = self.map_generated2[:-1]

        self.map_generated2 = np.vstack((self.map_generated2,np.array([0 , 0 , 1])))

    def generate_back(self,current_pose): #잠깐동안 후진하도록 만듦.
        cur_pos = [current_pose.x , current_pose.y]

        first_point = self.import_module.coord_conversion(cur_pos,self.parking_heading[0],[self.forback])

        first_point = np.vstack((first_point , cur_pos))
        

        self.back_waypoint = self.import_module.spline_interpolation(first_point)
        self.back_waypoint = self.back_waypoint[::-1]
        self.back_waypoint= np.vstack((self.back_waypoint,np.array([0, 0 , -1])))
        

    def backmoment(self,current_pose):
        if self.back_check==0:
            self.generate_back(current_pose)
            self.back_check +=1

        self.visualize_delivery_path(current_pose, self.back_waypoint)
            
        self.pub_delivery_path(self.back_waypoint)

        dist = sqrt((current_pose.x - self.back_waypoint[-2][0])**2 + (current_pose.y - self.back_waypoint[-2][1])**2)

        print(f'dist : {dist}')

        if dist <= 1.2:
            self.change_delivery_status()
        

    def back2glob(self,current_pose):
        if self.reverse_check==0:
            self.reverse_delivery_path()
            self.reverse_check +=1
       
        self.visualize_delivery_path(current_pose, self.map_generated2)
            
        self.pub_delivery_path(self.map_generated2)

        dist = sqrt((current_pose.x - self.map_generated2[-2][0])**2 + (current_pose.y - self.map_generated2[-2][1])**2)

        print(f'dist : {dist}')
        print(self.map_generated2)

        if dist <= 1.5 :
            self.change_delivery_status()
        

    def point_division(self,start_point,end_point,angle_diff,mode):
        
        points=[start_point]
        
        if mode==0: #arc1
            angle=angle_diff/3
            for index in range(1,3,1):
                
                points.append([start_point[0] + self.R_turn * np.sin(angle*index) , 
                               start_point[1] + (self.R_turn- self.R_turn*np.cos(angle*index))])

            points.append(end_point)
            
            return points
        
        elif mode==1: #line
            
            points.append(end_point)
            
            return points
            
        elif mode==2: #arc2
            angle = angle_diff/3
            for index in range(2,0,-1):

                points.append([end_point[0] -  (self.R_turn + 0.3) * np.sin(angle*index), end_point[1] - (self.R_turn + 0.3- (self.R_turn +0.3) * np.cos(angle*index)) ])
                
            points.append(end_point)
            return points 
        
    def pub_delivery_path(self, waypoints):
        self.delivery_path = PointCloud()
        for point in waypoints:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            self.delivery_path.points.append(p)
        self.delivery_pub.publish(self.delivery_path)

    def visualize_delivery_path(self, current_pose, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='macaron', stamp=rospy.get_rostime()),
            ns="delivery_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.2,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0.0,b=1.0,a=0.8)
        )
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0] - current_pose.x
            p.y = waypoint[1] - current_pose.y
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.vizpath_pub.publish(rviz_msg_path)

    def pub_endsignal(self):
        end_path = np.array([0 , 0 , 0])
        end_path = np.vstack((end_path , np.array([0 , 0 , 0])))

        self.pub_delivery_path(end_path)