#!/usr/bin/env python
#-*-coding:utf-8-*-

# Python packages

import rospy
import time
import numpy as np
# message 파일
from macaron_5.msg import erp_write

def main():
    while not rospy.is_shutdown():
        erp = erp_write()
        erp.write_steer = 0
        erp.write_brake = 0
        erp_pub.publish(erp)
        
    

if __name__ == '__main__':
    erp_pub = rospy.Publisher("speed_planner", erp_write, queue_size=1)
    rospy.init_node("pub_node", anonymous=True)
    main()