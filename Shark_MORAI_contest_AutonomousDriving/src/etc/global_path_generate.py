#!/usr/bin/env python3
#-*-coding:utf-8-*-

import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/sensor")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")

import matplotlib.pyplot as plt
from math import *
import numpy as np
import global_path
import rospy



import time
from pyproj import Proj, transform

from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import GPSMessage


#####################################################################

# mode 0 : Enter키로 node-> txt로 저장하고 node를 통해 만든 global_path는 npy로 저장
# mode 1 : 저장된 node txt파일을 불러들여서 global_path npy로 저장

mode = 1

WHERE = 6

#####################################################################

PATH_ROOT_TXT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/txt_file/"
PATH_ROOT_NPY=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/"

######################################################################

# GLOBAL_NPY = "yyy"     # 알아서 npy_file/path 에 저장됨
GLOBAL_TXT = "morai_2.txt"#"kcity_main_2023.txt" # mode 0 : 저장할 txt 파일 이름, mode 1 : 저장된 txt파일 이름
GLOBAL_NPY = "morai_2_gpg"#"kcity_main_2023"


######################################################################

index = 0
way_point = np.empty((1,2))

proj_UTMK = Proj(init='epsg:5179')
proj_WGS84 = Proj(init='epsg:4326')


class position:
    def __init__(self):
        # self.subtm = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.tm,queue_size=1)
        self.subtm = rospy.Subscriber("/gps", GPSMessage, self.tm, queue_size=1)
        self.pos = [0,0]

    def tm(self,Fix):
        lon = Fix.longitude
        lat = Fix.latitude
        x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
        self.pos = [x,y]



class GenerateWayPoints:
    def __init__(self, point1, point2, direction):
        self.x1, self.y1 = point1
        self.x2, self.y2 = point2

        self.parameter = abs(direction)  # < parameter = 0 : 직선 >    < parameter = 0 ~ 1 : 원호 >   < parameter = 1 ~ : 타원 >
        self.data_length = 10  # 얼마나 촘촘 하게 나눌 것인지
        self.direction = direction  # 1 or -1

    def get_center_point(self):
        return 0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2)

    def get_slope(self):
        return atan2(self.y2 - self.y1, self.x2 - self.x1) + pi / 2

    def distance_two_points(self):
        return sqrt((self.x2 - self.x1) ** 2 + (self.y2 - self.y1) ** 2)

    def generate_linear_point(self):
        x = np.linspace(self.x1, self.x2, self.data_length)
        y = np.linspace(self.y1, self.y2, self.data_length)
        return np.column_stack((x, y))

    def generate_circular_point(self):
        center_x, center_y = self.get_center_point()
        theta = self.get_slope()
        radius = sqrt((self.x1 - center_x) ** 2 + (self.y1 - center_y) ** 2) * (1 / self.parameter)
        theta_prime = acos(1 - 0.5 * self.distance_two_points() ** 2 / radius ** 2)

        circle_x, circle_y, t = 0, 0, None
        if self.direction >= 0:
            circle_x = center_x - radius * cos(theta_prime / 2) * cos(theta)
            circle_y = center_y - radius * cos(theta_prime / 2) * sin(theta)
            t = np.linspace(theta - theta_prime / 2, theta + theta_prime / 2, self.data_length)
        elif self.direction < 0:
            circle_x = center_x + radius * cos(theta_prime / 2) * cos(theta)
            circle_y = center_y + radius * cos(theta_prime / 2) * sin(theta)
            t = np.linspace(pi + theta - theta_prime / 2, pi + theta + theta_prime / 2, self.data_length)
        else:
            pass
        x = circle_x + radius * np.cos(t)
        y = circle_y + radius * np.sin(t)
        return np.column_stack((x, y))

    def generate_elliptical_point(self):
        center_x, center_y = self.get_center_point()
        theta = self.get_slope()
        minor_axis = sqrt((self.x1 - center_x) ** 2 + (self.y1 - center_y) ** 2)
        major_axis = self.parameter * minor_axis

        t = None
        if self.direction <= -1:
            t = np.linspace(-pi / 2, pi / 2, self.data_length)
        elif self.direction >= 1:
            t1 = np.linspace(pi / 2, pi, self.data_length // 2)
            t2 = np.linspace(-pi, -pi / 2, self.data_length // 2)
            t2 = np.delete(t2 , 0) # 중복된 값 제거
            t = np.concatenate((t1, t2))
        else:
            pass

        x = center_x - major_axis * np.cos(t) * np.cos(theta) - minor_axis * np.sin(t) * np.sin(theta)
        y = center_y - major_axis * np.cos(t) * np.sin(theta) + minor_axis * np.sin(t) * np.cos(theta)
        return np.column_stack((x, y))

    def generate_way_points(self):
        way_points = None
        if self.parameter == 0:
            way_points = self.generate_linear_point()
        elif 0 < self.parameter <= 1:
            way_points = self.generate_circular_point()
        elif 1 < self.parameter:
            way_points = self.generate_elliptical_point()
        else:
            pass

        if round(way_points[0][0], 2) != round(self.x1, 2) or round(way_points[0][1], 2) != round(self.y1, 2):
            way_points = np.flipud(way_points)

        # way_points = way_points.tolist()
        print("count")
        return way_points


def main():
    global way_point
    gp_name_npy = PATH_ROOT_NPY+GLOBAL_NPY
    flag = 0
    rospy.init_node('global_path_generator',anonymous=True)
    if(mode == -1) :

        x = []
        y = []
        pre_x = []
        pre_y = []
        last_x = 0
        last_y = 0

        gp_name = PATH_ROOT_TXT + GLOBAL_TXT

        f = open(gp_name, 'w')

        last_point = list(map(float, input('시작 좌표 : ').split(' ')))

        print("x : {}, y: {}".format(last_point[0] , last_point[1]))

        plt.plot(last_point[0], last_point[1], "r*", zorder = 3)
        while True :
            try :
                flag=int(input("Press num1 Key to save next point (end -> -201 + EnterKey) : "))
            except :
                pass
            if(flag == -201) :
                x.append(last_x)
                y.append(last_y)
                break

            point1 = list(map(float, input('좌표 : ').split(' ')))

            print("x : {}, y: {}".format(point1[0] , point1[1]))

            direction = float(input('k(-: 좌회전, +: 우회전) : '))

            data = "{} {} {} {} {}\n".format(last_point[0], last_point[1], point1[0], point1[1], direction)

            print(last_point, point1)

            wp = GenerateWayPoints(last_point, point1, direction)

            way_points = wp.generate_way_points()

            x_list, y_list = list(map(lambda x: list(x), zip(*way_points)))

            pre_x.extend(x_list)
            pre_y.extend(y_list)

            glob_path = global_path.GlobalPath(x=pre_x,y=pre_y)

            pre_way_point = np.zeros((1,2))

            for i in range(len(glob_path.rx)) :
                pre_way_point = np.append(pre_way_point,np.array([[glob_path.rx[i], glob_path.ry[i]]]), axis=0)

            pre_way_point = np.delete(pre_way_point, (0), axis=0)

            np.save(gp_name_npy, pre_way_point)

            check_flag = int(input("revert: -1, apply : 1 -> "))

            if(check_flag == 1) :
                last_x = x_list.pop()
                last_y = y_list.pop()

                x.extend(x_list)
                y.extend(y_list)
                pre_x.pop()
                pre_y.pop()

                plt.plot(point1[0], point1[1], "r*",zorder = 3)

                last_point = point1
                f.write(data)
            elif(check_flag == -1) :

                pre_x = x
                pre_y = y

                glob_path = global_path.GlobalPath(x=pre_x,y=pre_y)

                pre_way_point = np.zeros((1,2))

                for i in range(len(glob_path.rx)) :
                    pre_way_point = np.append(pre_way_point,np.array([[glob_path.rx[i], glob_path.ry[i]]]), axis=0)

                pre_way_point = np.delete(pre_way_point, (0), axis=0)

                np.save(gp_name_npy, pre_way_point)

        f.close()

        plt.plot(glob_path.rx, glob_path.ry, '.')
        plt.axis('equal')
        plt.show()

    if(mode == 0) :
        x = []
        y = []
        pre_x = []
        pre_y = []
        last_x = 0
        last_y = 0

        p = position()

        gp_name = PATH_ROOT_TXT + GLOBAL_TXT

        f = open(gp_name, 'w')

        input("Press Enter Key to save start point")

        last_point = p.pos

        print("x : {}, y: {}".format(last_point[0] , last_point[1]))

        plt.plot(last_point[0], last_point[1], "r*", zorder = 3)
        while True :
            try :
                flag=int(input("Press Enter Key to save next point (end -> -201 + EnterKey) : "))
            except :
                pass
            if(flag == -201) :
                x.append(last_x)
                y.append(last_y)
                break

            point1 = p.pos

            print("x : {}, y: {}".format(point1[0] , point1[1]))

            direction = float(input('k(-: 좌회전, +: 우회전) : '))

            data = "{} {} {} {} {}\n".format(last_point[0], last_point[1], point1[0], point1[1], direction)

            wp = GenerateWayPoints(last_point, point1, direction)

            way_points = wp.generate_way_points()

            x_list, y_list = list(map(lambda x: list(x), zip(*way_points)))

            pre_x.extend(x_list)
            pre_y.extend(y_list)

            glob_path = global_path.GlobalPath(x=pre_x,y=pre_y)

            pre_way_point = np.zeros((1,2))

            for i in range(len(glob_path.rx)) :
                pre_way_point = np.append(pre_way_point,np.array([[glob_path.rx[i], glob_path.ry[i]]]), axis=0)

            pre_way_point = np.delete(pre_way_point, (0), axis=0)

            np.save(gp_name_npy, pre_way_point)

            check_flag = int(input("revert: -1, apply : 1 -> "))

            if(check_flag == 1) :
                last_x = x_list.pop()
                last_y = y_list.pop()

                x.extend(x_list)
                y.extend(y_list)
                pre_x.pop()
                pre_y.pop()

                plt.plot(point1[0], point1[1], "r*",zorder = 3)

                last_point = point1
                f.write(data)
            elif(check_flag == -1) :

                pre_x = x
                pre_y = y

                glob_path = global_path.GlobalPath(x=pre_x,y=pre_y)

                pre_way_point = np.zeros((1,2))

                for i in range(len(glob_path.rx)) :
                    pre_way_point = np.append(pre_way_point,np.array([[glob_path.rx[i], glob_path.ry[i]]]), axis=0)

                pre_way_point = np.delete(pre_way_point, (0), axis=0)

                np.save(gp_name_npy, pre_way_point)

        f.close()

        plt.plot(glob_path.rx, glob_path.ry, '.')
        plt.axis('equal')
        plt.show()

    if mode == 1:
        x = []
        y = []
        last_x = 0
        last_y = 0

        gp_name = PATH_ROOT_TXT + GLOBAL_TXT

        f = open(gp_name, 'r')

        lines = f.readlines()

        for line in lines :
            line = line.strip()

            ls = line.split(' ')

            plt.plot(float(ls[0]), float(ls[1]), "r*",zorder = 3)
            plt.plot(float(ls[2]), float(ls[3]), "r*",zorder = 3)

            wp = GenerateWayPoints([float(ls[0]), float(ls[1])],[float(ls[2]), float(ls[3])], float(ls[4]))

            way_points = wp.generate_way_points()

            x_list, y_list = list(map(lambda x: list(x), zip(*way_points)))

            last_x = x_list.pop()
            last_y = y_list.pop()

            x.extend(x_list)
            y.extend(y_list)

        x.append(last_x)
        y.append(last_y)

        glob_path = global_path.GlobalPath(x=x,y=y,ds=0.5)


        f.close()

        for i in range(len(glob_path.rx)) :
            way_point = np.append(way_point,np.array([[glob_path.rx[i], glob_path.ry[i]]]), axis=0)

        way_point = np.delete(way_point, (0), axis=0)

        gp_name_npy = PATH_ROOT_NPY+GLOBAL_NPY

        np.save(gp_name_npy, way_point)

        plt.plot(glob_path.rx, glob_path.ry, '.')
        plt.axis('equal')
        plt.show()

if __name__ == '__main__':
    main()