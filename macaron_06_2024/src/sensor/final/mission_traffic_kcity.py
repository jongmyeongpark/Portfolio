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

class mission_traffic_kcity:
    def __init__(self):
        self.stop_s = [0,        # start
                        230.1,   # 본선 1
                        283.1,   # 본선 2
                        420.1,   # 본선 3
                        527.1,   # 본선 4
                        605.9,   # 본선 5
                        903.2,   # 본선 6
                        949.3]   # 본선 7


                     
        self.Straight_Stop_Line = [self.stop_s[2], self.stop_s[3], self.stop_s[6], self.stop_s[7]]
        self.Left_Stop_Line = [self.stop_s[1], self.stop_s[4], self.stop_s[5]]

        self.sub_sign = rospy.Subscriber('/traffic_obj_combined', Traffic, self.obj_callback, queue_size=1)
        self.pub_sign = rospy.Publisher('/stop', Bool, queue_size=1)
        self.pub_sign_state = rospy.Publisher('/stop_sign', String, queue_size=1)
        
        # self.current_s_sub = rospy.Subscriber('/current_s', Float32, self.s_callback, queue_size=1)

        self.go = 0
        self.stop = 0
        self.turn_left = 0
        self.traffic_s = 99999999
        self.done = False
        self.Stop_State = 'No Detect'
        self.mode = 0
        self.traffic_flag = 0
        self.current_s = 0.0
        self.end_time = None
        self.signal_flag = None
        self.green_signal = 0.0 
        self.traffic_signal_list = []
        self.time_diff = 0

        self.pass_count = 0 

        self.s_count =self.stop
        self.l_count = self.turn_left
        self.g_count = self.go 

    def s_callback(self, s):
        self.current_s = s.data


    def obj_callback(self, data):
        for sign in data.obj:
            self.traffic_update(sign.ns)

    def traffic_update(self, ns):
        if ns == "red":
            self.stop += 1
            self.go = 0
            self.turn_left = 0
        elif ns == "yellow" or ns == "green" or ns == "all_green":
            self.stop = 0
            self.go += 1
            if ns == "all_green":
                self.turn_left += 1
        elif ns == "left":
            self.stop = 0
            self.go = 0
            self.turn_left += 1

    def reset(self):
        self.stop = 0
        self.turn_left = 0
        self.go = 0    

        self.pass_count = 0 

        self.traffic_s = 99999999
        
        self.end_time = None
        self.Stop_State = 'No detect'
        self.traffic_flag = 0

    def run(self, s):
        self.current_s = s
        print(self.time_diff)
        # s = self.current_s
        detect_max_gap = 7
        candidate_stop_s = []
        # print(f"self.turn_left : {self.turn_left}")
        # print(f"self.go : {self.go}")
        # print(f"self.stop : {self.stop}")
        
        for stop in self.stop_s:
            if self.current_s <= stop and self.current_s > 0:
                candidate_stop_s.append(stop)
                
                
        print(f"current_s : {self.current_s}")
        #print(f"candidate_stop_s: {candidate_stop_s}")
        if len(candidate_stop_s) > 0:
            candidate_stop_s.sort()
            self.traffic_s = candidate_stop_s[0]

        if 7 < self.traffic_s - self.current_s:
            print(f"distance :{self.traffic_s - self.current_s}")
            self.Stop_State = 'distance: ' + str(self.traffic_s - self.current_s)
            self.publish()
            return 

        if self.traffic_flag == 0:
            if 0 < self.traffic_s - self.current_s <= detect_max_gap:
                if self.traffic_s in self.Straight_Stop_Line:
                    self.mode = 0  
                elif self.traffic_s in self.Left_Stop_Line:
                    self.mode = 1 
                self.traffic_flag = 1

        elif self.traffic_flag == 1:    
            if self.mode == 0:  
                print(f"self.Stop_State : {self.Stop_State}")
                print(f"self.traffic_flag : {self.traffic_flag}")   
                if self.traffic_s == self.stop_s[6] :
                    if self.time_diff > 800:
                        self.Stop_State = 'Go'  
                    else:
                        if self.go > 3:
                            self.Stop_State = 'Go'                  
                            
                        elif self.stop > 3 or self.turn_left > 3:
                            self.Stop_State = 'Stop'

                elif self.traffic_s == self.stop_s[7] :
                    if self.time_diff > 840:
                        self.Stop_State = 'Go'  
                    else:
                        if self.go > 3:
                            self.Stop_State = 'Go'                  
                            
                        elif self.stop > 3 or self.turn_left > 3:
                            self.Stop_State = 'Stop'
                    
                elif self.go > 3:
                    self.Stop_State = 'Go'                  
                
                elif self.stop > 3 or self.turn_left > 3:
                    self.Stop_State = 'Stop'

            elif self.mode == 1:        
                if self.traffic_s == self.stop_s[4] :
                    print(f'self.go: {self.go}')
                    if self.turn_left > 3:
                        self.Stop_State = 'Go'                 
                        
                    elif self.go > 25:
                        # pass_count 파라미터 수정 필요
                        print(f'self.pass_count: {self.pass_count}')
                        if self.pass_count > 25:
                            self.Stop_State = 'Go'
                        
                        else:
                            if (self.stop == self.s_count) and (self.turn_left == self.l_count) and (self.go == self.g_count):
                                self.Stop_State = 'Stop'
                                self.pass_count += 1


                            else:
                                self.Stop_State = 'Stop'
                                self.s_count =self.stop
                                self.l_count = self.turn_left
                                self.g_count = self.go 


                    else:
                        self.Stop_State = 'Stop'
                                                                                        
                
                elif self.turn_left > 3:
                    self.Stop_State= 'Go'
                    if self.traffic_s == self.stop_s[1]:
                        self.traffic_signal_list.append(0)
                
                elif self.stop > 3 or self.go > 3:
                    self.Stop_State = 'Stop'
                    if self.traffic_s == self.stop_s[1]:
                        self.traffic_signal_list.append(1)


                if self.traffic_s == self.stop_s[1]:
                    if self.signal_flag == None:
                        if self.traffic_s == self.stop_s[1]:
                            if self.traffic_signal_list == [0]:
                                self.green_signal = time.time()
                                self.signal_flag = 'Similiar'
                                # self.stop_s[6] += 11
                    
                            elif len(self.traffic_signal_list) > 1:
                                if str(self.traffic_signal_list[-1]) == str(0):
                                    self.green_signal = time.time()
                                    self.signal_flag = 'Exact'
                        
                        #print(self.signal_flag)
                  
            
            if self.traffic_s - self.current_s <= 1.0:
                self.traffic_flag = 2

        elif self.traffic_flag == 2:
            if self.end_time is None:
                self.end_time = time.time()
            
            if self.mode == 0 and self.Stop_State == 'Stop' and time.time() < self.end_time + 100:
                if self.go > 3:
                    self.traffic_flag = 3
                    self.Stop_State= 'Go'
            elif self.mode == 1 and self.Stop_State == 'Stop' and time.time() < self.end_time + 100:
                if self.turn_left > 3:
                    self.traffic_flag = 3
                    self.Stop_State= 'Go'
            else:
                self.traffic_flag = 3

        elif self.traffic_flag == 3:
            self.Stop_State = 'Finish'
            self.reset()

        if self.traffic_s - self.current_s <= 0.5:
            self.reset()     
        
        self.publish()
        self.pub_sign_state.publish(self.Stop_State)
     
    def publish(self):
        if self.Stop_State == 'Stop' or self.Stop_State == 'Time out':
            self.pub_sign.publish(True)
        
        else: 
            self.pub_sign.publish(False)
        rospy.loginfo(f"정지 상태 여부: {self.Stop_State}")

def main():
    rospy.init_node('traffic_node')
    rate = rospy.Rate(10)
    traffic = mission_traffic_kcity()
    while not rospy.is_shutdown():
        traffic.run()
        rate.sleep()
        
        
if __name__ == "__main__":
    main()