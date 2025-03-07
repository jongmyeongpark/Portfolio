#!/usr/bin/env python
# -*-coding:utf-8-*-

import os, sys
import rospy

import numpy as np
import matplotlib.pyplot as plt
import time

from macaron_5.msg import erp_read, erp_write

trace = np.empty((1, 3))

class erp_info:
    def __init__(self):
        self.erp_sub= rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        
        self.erp_pub = rospy.Publisher("speed_planner", erp_write, self.erp_callback2, queue_size=1)
        self.current_speed = 0
        self.target_speed = 0
        self.mode = False

    def erp_callback(self, data):
        self.current_speed = data.read_speed
        self.mode = data.read_AorM

    def erp_callback2(self, data):
        self.target_speed = data.write_speed

def main():
    global trace
    rospy.init_node('visual_speed', anonymous=True)
    start_time = None
    erp = erp_info()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        if erp.mode:
            
            if start_time == None and erp.current_speed > 1:
                start_time = time.time()
                
            if start_time != None:
                s = erp.current_speed
                s2 = erp.target_speed
                t = time.time() - start_time
                trace = np.append(trace, np.array([[t, s, s2]]), axis=0)
                print('time, speed : ', t, s, s2)
        
        rate.sleep()

    trace = np.delete(trace, 0, axis=0)
    
    plt.plot(trace[:, 0], trace[:, 1], 'red', trace[:, 0], trace[:, 2], 'green')
    
    plt.show()


if __name__ == "__main__":
    main()
