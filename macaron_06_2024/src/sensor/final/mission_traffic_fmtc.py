#!/usr/bin/env python3
# -*-coding:utf-8-*-
import rospy
import sys, os
import time
import numpy as np
from macaron_06.msg import Traffic, obj_info
from std_msgs.msg import Bool, String, Float32
from ublox_msgs.msg import NavPVT   
from std_msgs.msg import Bool

# names: {0: 'green', 1: 'yellow', 2: 'red', 3: 'all_green', 4: 'left', 5: 'delivery_a1', 6: 'delivery_a2', 7: 'delivery_a3', 8: 'delivery_b1', 9: 'delivery_b2', 10: 'delivery_b3'}

# MODE = 0  # 미션에 따라 모드 설정해주기(0:직진 / 1:좌회전)
class mission_traffic:
    def __init__(self):
        #fmtc
        
        # rospy.init_node('mission_traffic_node') 
        # if Place==1:
        #    self.Straight_Stop_Line =  []
        #    self.Left_Stop_Line = []

        
        # #k_city
        # if Place==2:
        self.stop_s = [0,    # start
                        30,  # 본선 1
                        280,   # 본선 2
                        417,   # 본선 3
                        525.3,   # 본선 4
                        578.8,   # 본선 5
                        898.2, # 본선 6
                        946.2] # 본선 7
                    
                     
                       
        self.Straight_Stop_Line = [self.stop_s[2],self.stop_s[3],self.stop_s[6], self.stop_s[7]]
        self.Left_Stop_Line = [self.stop_s[1],self.stop_s[4],self.stop_s[5]]

        
        self.sub_sign = rospy.Subscriber('/traffic_obj_combined', Traffic, self.obj_callback, queue_size=1)
        self.pub_sign = rospy.Publisher('/stop', Bool, queue_size=1)
        # self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.gps_callback, queue_size=1)
        
        self.go = 0
        self.stop = 0
        self.turn_left = 0
        self.traffic_s = 99999999
        
        self.done = False
        self.Stop_State = 'No Detect'
        
        self.start_count = 0
        self.go_count = 0
        self.stop_count = 0
        self.turn_left_count = 0

        self.mode = 0
        self.traffic_flag = 0
        
        self.interval_time_straight = 100
        self.interval_time_left = 100
        self.current_s = 0.0
        self.end_time = None
        self.go_flag = None   

        self.signal_flag = None
        self.green_signal = 0.0 
        self.traffic_signal_list = []
    
        self.current_s = None
        
    def obj_callback(self, data):
        for sign in data.obj:
            self.traffic_update(sign.ns)
    
    def traffic_update(self, ns):
        if ns == "red":
            self.stop += 1
            self.go = 0
            self.turn_left = 0

        elif ns == "yellow":
            self.stop = 0
            self.go += 1
            self.turn_left = 0
 

        elif ns == "green":
            self.stop = 0
            self.go += 1
            self.turn_left = 0


        elif ns == "all_green":
            self.stop = 0
            self.go += 1
            self.turn_left += 1


        elif ns == "left":
            self.stop = 0
            self.go = 0
            self.turn_left += 1


    def reset(self):
        self.stop = 0
        self.turn_left = 0
        self.go = 0    

        self.start_count = 0
        self.go_count = 0
        self.stop_count = 0
        self.turn_left_count = 0

        self.go_flag = None
        self.done = False
        
        # self.Stop_State = 'No Detect'

    def reset_count(self):
        self.stop = 0
        self.turn_left = 0
        self.go = 0
   
    def run(self, s):
        start_time = time.time()
        self.current_s = s
        
        #인지 판단 구간 변수
        detect_max_gap = 35
        detect_min_gap = 0

        candidate_stop_s = []
        
        for stop in self.stop_s:
            if self.current_s <= stop and self.current_s > 0:
                candidate_stop_s.append(stop)
        if len(candidate_stop_s) > 0:
                candidate_stop_s.sort()
                min_arr = candidate_stop_s
                min_value = min_arr[0]
                # print('min value: ', min_value)
                self.traffic_s = min_value

        # print(f"distance :{self.traffic_s - self.current_s}")

        if 35 < self.traffic_s - self.current_s < 50:
            print(f"distance :{self.traffic_s - self.current_s}")
            distance = self.traffic_s - self.current_s
            self.Stop_State = 'distance: ' + str(distance)

        # 신호등 인식 시작 구간 진입 여부 판단
        if self.traffic_flag == 0:
            # # 인지판단 구간(정지선 기준 15m 앞 ~ 3m 앞)
            if (detect_min_gap < self.traffic_s - self.current_s < detect_max_gap):
                if  self.traffic_s in self.Straight_Stop_Line:
                    self.mode = 1  
                elif self.traffic_s in self.Left_Stop_Line:
                    self.mode = 0 
   
 
                #print('정지에 대비하기 위해 감속 합니다!')
                self.traffic_flag = 1

        # 신호등 인식 구간 진입 및 판단
        elif self.traffic_flag == 1:           
            '''
            1. detect 횟수 세기(연속으로 detect_num만큼 detect하면 count +1하기)
            2. count 횟수 세기(연속으로 count_num 만큼 count 횟수가 늘어나면 최종판단)
            '''        
            # 직진인 경우
            if self.mode == 0:
                # print('stop cnt :' ,self.stop_count)
                # print('stop : , ',self.stop)
                # print('go cnt  : ', self.go_count)
                # print('go : ' ,self.go)
                # print('turn left : ' ,self.turn_left)
                # print('turn left cnt: ' ,self.turn_left_count)
                if self.go > 3:
                    self.stop_count = 0
                    self.go_count += 1
                    self.turn_left_count = 0
                    #self.reset_count()                    

                # 빨간불(red) or 주황불(yellow)
                elif self.stop > 3:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    #self.reset_count()
                    
                # 좌회전(left)
                elif self.turn_left > 3:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    #self.reset_count()
  
                if self.go_count > 3:
                    self.Stop_State = 'Go'
                    #self.reset()
 
                
                elif self.stop_count > 3:
                    self.Stop_State = 'Stop'
                    self.publish()
                    #self.reset()

                
                elif self.turn_left_count > 3:
                    self.Stop_State = 'Stop'
                    self.publish()
                    #self.reset()

                                 
                if self.traffic_s == self.stop_s[6] :
                    self.go_interval = time.time()
                    x = (self.go_interval - self.green_signal)
                    print('time diff: ', x)
                    if self.signal_flag == 'Similiar':
                        cycle = x // 100
                        if 100 * cycle + 38 <= x <= 100* cycle + 38 + 12:
                            self.Stop_State = 'Time in'
                            # return 'Time in'
                        else:
                            self.Stop_State = 'Time out'
                            # return 'Time out'
                            # self.traffic_flag = 3 
                            
                                                                         
                    elif self.signal_flag == 'Exact':
                        cycle = x // 100
                        if 100 * cycle + 50 - 10 <= x <= 100* cycle + 50 + 10:
                            self.Stop_State = 'Time in'
                            # return 'Time in'
                        else:
                            self.Stop_State = 'Time out'
                            self.publish()
                            # return 'Time out'
                            # self.traffic_flag = 3 

                if self.traffic_s == self.stop_s[7]:
                    self.go_interval = time.time()
                    x = (self.go_interval - self.green_signal)
                    print('time diff: ', x)
                    if self.signal_flag == 'Similiar':
                        cycle = x // 100
                        if 100 * cycle + 38 <= x <= 100* cycle + 38 + 12:
                            self.Stop_State = 'Time in'
                            # return 'Time in'
                        else:
                            self.Stop_State = 'Time out'
                            # return 'Time out'
                            # self.traffic_flag = 3 
                    
                    elif self.signal_flag == 'Exact':
                        cycle = x // 100
                        if 100 * cycle + 50 - 10 <= x <= 100* cycle + 50 + 10:
                            self.Stop_State = 'Time in'
                            # return 'Time in'
                        else:
                            self.Stop_State = 'Time out'
                            # return 'Time out'
                            # self.traffic_flag = 3 


            # 좌회전 미션인 경우
            elif self.mode == 1:
                # 좌회전(left) 
                if self.turn_left > 3:
                    self.stop_count = 0
                    self.go_count = 0
                    self.turn_left_count += 1
                    self.reset_count()

                # 빨간불(red), 주황불(orange)
                elif self.stop > 3:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    self.reset_count()

                # 초록불(green, all_green)
                elif self.go > 3:
                    self.stop_count = 0
                    self.go_count += 1
                    self.turn_left_count = 0
                    self.reset_count()

                if self.turn_left_count >= 3:
                    self.Stop_State= 'Go'
                    self.traffic_signal_list.append(0)
                    self.publish()
                    self.reset()


                elif self.go_count > 3:
                    self.Stop_State = 'Stop'
                    self.traffic_signal_list.append(1)
                    self.publish()
                    self.reset()

                
                elif self.stop_count > 3:
                    self.Stop_State = 'Stop'
                    self.traffic_signal_list.append(1)
                    self.publish()
                    self.reset()


                
                if self.signal_flag == None:# 예시
                    #print(self.traffic_signal_list)
                    if self.traffic_s == self.stop_s[1]:
                        if self.traffic_signal_list == [0]:
                            self.green_signal = time.time()
                            self.signal_flag = 'Similiar'
                            
                        elif len(self.traffic_signal_list) > 1:
                             if str(self.traffic_signal_list[-1]) == str(0):
                                self.green_signal = time.time()
                                self.signal_flag = 'Exact'
                                #print('111111111111111111111111111111111111')
                    print(self.signal_flag)
            
            over_stop_line = 2.0
            if self.traffic_s - self.current_s <= over_stop_line:  # 인지판단 구간을 넘을 시
                self.traffic_flag = 2

                if self.Stop_State is True:
                    print("정지 신호")

                else:
                    print("직진 신호 혹은 좌회전 신호")
       
        elif self.traffic_flag == 2:
            if self.end_time is None:
                self.end_time = time.time()

            if self.mode == 0:
                if self.Stop_State == 'Stop' and time.time() < self.end_time + self.interval_time_straight :
                    # start_count 3번 연속으로 나오면 Go라고 신호 보내기
                    if self.go > 1:
                        self.start_count += 1
                        print(f"self.start_count: {self.start_count}")
                        self.publish()
                        self.reset_count()
                    else:
                        pass

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                        self.Stop_State= 'Go'
                        self.publish()
                        self.reset()


                else:
                    self.traffic_flag = 3                
                    
            elif self.mode == 1:
                
                if self.Stop_State is True and time.time() < self.end_time + self.interval_time_left :
                   # start_count 3번 연속으로 나오면 Go라고 신호 보내기
                    if self.turn_left > 1:
                        self.start_count += 1
                        self.reset_count()
                    else:
                        self.start_count = 0

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                        self.Stop_State= 'Go'
                        self.publish()
                        self.reset()

                                        
                else:
                    self.traffic_flag = 3

            self.time_rec = time.time()
                              

        # 초록불 판단 후
        elif self.traffic_flag == 3:
            self.done = True
            self.Stop_State = 'Finish'
            self.reset()
            #print('신호등 미션 끝!\n')
            self.end_time = None
            self.traffic_flag = 0
        end_time = time.time()

        print('elapsed time: ', end_time - start_time)

    def publish(self):
        if self.Stop_State == "Stop" or self.Stop_State == "Time out":
            #print("publish info: :", self.Stop_State)
            self.pub_sign.publish(True)
                                  


# def main():
#     traffic = mission_traffic()

#     while not rospy.is_shutdown():
    
#         current_s = traffic.current_s

#         # Check if we are near any of the stop lines
#         if any([-5 <= (line - current_s) <= 55 for line in traffic.Straight_Stop_Line]):
#             traffic.run(0)
#         elif any([-5 <= (line - current_s) <= 55  for line in traffic.Left_Stop_Line]):
#             traffic.run(1)
#         else:
#             pass
            

# if __name__ == "__main__":
#     main()