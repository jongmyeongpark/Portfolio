#!/usr/bin/env python3
#-*-coding:utf-8-*-

# Python packages
import rospy
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")
import numpy as np
import time
from pyproj import Proj, transform
from math import pi
import math
from scipy.spatial import distance
import matplotlib.pyplot as plt
import bisect

from sensor_msgs.msg import NavSatFix


######################################################################
######################################################################
# GLOBAL_NPY = "yyy"     # 알아서 npy_file/path 에 저장됨
GLOBAL_NPY = "2024_kcity_pre_1019dfdfd.npy"
# GLOBAL_NPY = "kcity_deliver_0805.npy"
# GLOBAL_NPY = "kcity_pickup_0805.npy"
# GLOBAL_NPY = "snu_r_obs_0726.npy"
# GLOBAL_NPY = "snu_l_obs_0726.npy"
DOT_DISTANCE = 0.5                      # 점과 점 사이 거리 0.5m
######################################################################
######################################################################

proj_UTMK = Proj(init='epsg:5179')
proj_WGS84 = Proj(init='epsg:4326')

index = 0
waypoint = np.empty((1,2))
dt=0.1

flag = 0

class position:
    def __init__(self):
        self.subtm = rospy.Subscriber("smc_2000/fix", NavSatFix, self.tm,queue_size=1)
        self.pos = [0,0]

    def tm(self,Fix):
        lon = Fix.longitude
        lat = Fix.latitude
        x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
        self.pos = [x,y]

class Draw_map():
    def __init__(self):
        self.time_rec = time.time()
        self.recent_pose = [0,0]
        
    def rec_pose(self, pose):
        # Receive pose
        global index, waypoint, flag
        
        
        if np.hypot(self.recent_pose[0] - pose[0], self.recent_pose[1] - pose[1]) >= DOT_DISTANCE: # 0.5m 마다 찍음
            waypoint = np.append(waypoint, np.array([[pose[0], pose[1]]]), axis=0)

            dst = distance.euclidean(waypoint[1], waypoint[-1])
            print('distance', dst)
            self.recent_pose = pose
            

class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result
    
    def calcddd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        result = 6.0 * self.d[i]
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        #  print(B)
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y


def calc_spline_course(x, y, ds=0.1): # ds : 0.1 간격으로 점을 나눈다. 
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk, rdk = [], [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)

    return rx, ry

def main():
    global waypoint
    rospy.init_node('mapping',anonymous=True)
    p = position()
    d = Draw_map()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        d.rec_pose(p.pos)
        
        rate.sleep()
    waypoint = np.delete(waypoint, (0), axis=0)
    print(waypoint)

    # manhae1 = waypoint
    # x = manhae1[0:manhae1.shape[0]-1, 0]
    # y = manhae1[0:manhae1.shape[0]-1, 1]
    
    # rx, ry = calc_spline_course(x, y, DOT_DISTANCE)
    # r = []
    # for i in range(len(rx)):
    #     r.append([rx[i], ry[i]])
    # print(r)

    # 파일 경로 설정
    PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
    gp_name = PATH_ROOT + GLOBAL_NPY
    # np.save(gp_name, r)
    np.save(gp_name, waypoint)
    print('============save complete!============')
    plt.plot(waypoint[:, 0], waypoint[:, 1])
    plt.show()
    
if __name__ == "__main__":
    main()



# #!/usr/bin/env python
# #-*-coding:utf-8-*-

# # Python packages
# from pickle import GLOBAL
# import rospy
# import os, sys
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/sensor")
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")
# import numpy as np
# import time
# from pyproj import Proj, transform
# from math import pi
# import math
# from scipy.spatial import distance
# import matplotlib.pyplot as plt
# import bisect

# from sensor_msgs.msg import NavSatFix


# ######################################################################
# ######################################################################
# # GLOBAL_NPY = "K_city_track"     # 알아서 npy_file/path 에 저장됨
# GLOBAL_NPY = "snu"
# DOT_DISTANCE = 0.5                      # 점과 점 사이 거리 0.5m
# ######################################################################
# ######################################################################

# proj_UTMK = Proj(init='epsg:5179')
# proj_WGS84 = Proj(init='epsg:4326')

# index = 0
# waypoint = np.empty((1,2))
# dt=0.1

# flag = 0

# class position:
#     def __init__(self):
#         self.subtm = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.tm,queue_size=1)
#         self.pos = [0,0]

#     def tm(self,Fix):
#         lon = Fix.longitude
#         lat = Fix.latitude
#         x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
#         self.pos = [x,y]

# class Draw_map():
#     def __init__(self):
#         self.time_rec = time.time()
#         self.recent_pose = [0,0]
        
#     def rec_pose(self, pose):
#         # Receive pose
#         global index, waypoint, flag
        
        
#         if np.hypot(self.recent_pose[0] - pose[0], self.recent_pose[1] - pose[1]) >= DOT_DISTANCE: # 0.5m 마다 찍음
#             waypoint = np.append(waypoint, np.array([[pose[0], pose[1]]]), axis=0)

#             dst = distance.euclidean(waypoint[1], waypoint[-1])
#             print('distance', dst)
#             self.recent_pose = pose
            

# class Spline:
#     """
#     Cubic Spline class
#     """

#     def __init__(self, x, y):
#         self.b, self.c, self.d, self.w = [], [], [], []

#         self.x = x
#         self.y = y

#         self.nx = len(x)  # dimension of x
#         h = np.diff(x)

#         # calc coefficient c
#         self.a = [iy for iy in y]

#         # calc coefficient c
#         A = self.__calc_A(h)
#         B = self.__calc_B(h)
#         self.c = np.linalg.solve(A, B)
#         #  print(self.c1)

#         # calc spline coefficient b and d
#         for i in range(self.nx - 1):
#             self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
#             tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
#                 (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
#             self.b.append(tb)

#     def calc(self, t):
#         """
#         Calc position

#         if t is outside of the input x, return None

#         """

#         if t < self.x[0]:
#             return None
#         elif t > self.x[-1]:
#             return None

#         i = self.__search_index(t)
#         dx = t - self.x[i]
#         result = self.a[i] + self.b[i] * dx + \
#             self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

#         return result

#     def calcd(self, t):
#         """
#         Calc first derivative

#         if t is outside of the input x, return None
#         """

#         if t < self.x[0]:
#             return None
#         elif t > self.x[-1]:
#             return None

#         i = self.__search_index(t)
#         dx = t - self.x[i]
#         result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
#         return result

#     def calcdd(self, t):
#         """
#         Calc second derivative
#         """

#         if t < self.x[0]:
#             return None
#         elif t > self.x[-1]:
#             return None

#         i = self.__search_index(t)
#         dx = t - self.x[i]
#         result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
#         return result
    
#     def calcddd(self, t):
#         if t < self.x[0]:
#             return None
#         elif t > self.x[-1]:
#             return None

#         i = self.__search_index(t)
#         result = 6.0 * self.d[i]
#         return result

#     def __search_index(self, x):
#         """
#         search data segment index
#         """
#         return bisect.bisect(self.x, x) - 1

#     def __calc_A(self, h):
#         """
#         calc matrix A for spline coefficient c
#         """
#         A = np.zeros((self.nx, self.nx))
#         A[0, 0] = 1.0
#         for i in range(self.nx - 1):
#             if i != (self.nx - 2):
#                 A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
#             A[i + 1, i] = h[i]
#             A[i, i + 1] = h[i]

#         A[0, 1] = 0.0
#         A[self.nx - 1, self.nx - 2] = 0.0
#         A[self.nx - 1, self.nx - 1] = 1.0
#         #  print(A)
#         return A

#     def __calc_B(self, h):
#         """
#         calc matrix B for spline coefficient c
#         """
#         B = np.zeros(self.nx)
#         for i in range(self.nx - 2):
#             B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
#                 h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
#         #  print(B)
#         return B


# class Spline2D:
#     """
#     2D Cubic Spline class

#     """

#     def __init__(self, x, y):
#         self.s = self.__calc_s(x, y)
#         self.sx = Spline(self.s, x)
#         self.sy = Spline(self.s, y)

#     def __calc_s(self, x, y):
#         dx = np.diff(x)
#         dy = np.diff(y)
#         self.ds = [math.sqrt(idx ** 2 + idy ** 2)
#                    for (idx, idy) in zip(dx, dy)]
#         s = [0]
#         s.extend(np.cumsum(self.ds))
#         return s

#     def calc_position(self, s):
#         """
#         calc position
#         """
#         x = self.sx.calc(s)
#         y = self.sy.calc(s)

#         return x, y


# def calc_spline_course(x, y, ds=0.1): # ds : 0.1 간격으로 점을 나눈다. 
#     sp = Spline2D(x, y)
#     s = list(np.arange(0, sp.s[-1], ds))

#     rx, ry, ryaw, rk, rdk = [], [], [], [], []
#     for i_s in s:
#         ix, iy = sp.calc_position(i_s)
#         rx.append(ix)
#         ry.append(iy)

#     return rx, ry

# def main():
#     global waypoint
#     rospy.init_node('mapping',anonymous=True)
#     p = position()
#     d = Draw_map()

#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         d.rec_pose(p.pos)
        
#         rate.sleep()
#     waypoint = np.delete(waypoint, (0), axis=0)
#     print(waypoint)

#     # manhae1 = waypoint
#     # x = manhae1[0:manhae1.shape[0]-1, 0]
#     # y = manhae1[0:manhae1.shape[0]-1, 1]
    
#     # rx, ry = calc_spline_course(x, y, DOT_DISTANCE)
#     # r = []
#     # for i in range(len(rx)):
#     #     r.append([rx[i], ry[i]])
#     # print(r)

#     # 파일 경로 설정
#     PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
#     gp_name = PATH_ROOT + GLOBAL_NPY
#     # np.save(gp_name, r)
#     np.save(gp_name, waypoint)
#     print('============save complete!============')
#     plt.plot(waypoint[:, 0], waypoint[:, 1])
#     plt.show()
    
# if __name__ == "__main__":
#     main()
