#!/usr/bin/env python
# -*-coding:utf-8-*-

import rospy
import numpy as np
from sub_erp_state import sub_erp_state
from sensor_msgs.msg import PointCloud
import matplotlib.pyplot as plt

# 장애물 중복해서 보이는거 보기 좀 그러면 주석처리 해제하기.

erp = sub_erp_state()

def main():
    print('1111111111')
    print(erp.obs)
    obs = []
    # obs = set()
    try:
        while not rospy.is_shutdown():
            print('start recording')
            print(erp.pose)
            print(erp.obs)
            obs.append(np.array[erp.obs[0], erp.obs[1]])    
            # new_obs = tuple([erp.obs[0], erp.obs[1]])
            # obs.add(new_obs)
    except KeyboardInterrupt:
        print('pass')
        pass
    print('222222222')
    if obs:
        obs_array = np.array(obs)
        # obs_array = np.array(list(obs))
        obs_x = obs_array[:, 0]
        obs_y = obs_array[:, 1]
        print('3333333333')
        plt.figure()
        plt.scatter(obs_x, obs_y, color='red', marker='x', label='Obstacles')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()



