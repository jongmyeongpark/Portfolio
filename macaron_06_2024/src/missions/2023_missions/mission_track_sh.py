#!/usr/bin/env python
# -- coding: utf-8 --
import os, sys
import rospy
import math
import random
import time
import numpy as np
import copy
import scipy.interpolate as interp
from scipy.spatial.distance import cdist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16, Float32

sys.path.append(os.path.dirname(
    os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))) + "/src/path_planning")

from path_planning_tracking_dwa_PP import Path_Tracking_DWA
from global_path import GlobalPath

PT = Path_Tracking_DWA(file=1)
# 도로 폭m
offset = 2
# noise append 허용할 거리
noise_offset = 1.8
# 검출 성공 이라고 생각할 라바콘 개수
check_num = 3
# 추종 하고 싶은 거리m
look_distance = 2
# 선의 간격
line_offset = 0.3
# 보정치
coefficient = 1.0
# distance_filter 에서 사용, 콘 간의 최대, 최소 이격 가능 거리
min_dis = 1
max_dis = 5
# 란삭 반복 횟수, 임계 거리
num_it = 100
t_dis = 0.3

ex_follow_point = [2, 0]
ex_dwa_steer = 0
out_point_ransac = []
out_point_dis = []
append_noise = []
heading = 0
WB = 1.04
# 라이다부터 뒤축까지 거리m
back_wheel = 1.11
detect_flag = False
# 필터 키고 끄기 true=키기
filter_flag_r = False
filter_flag_d = False
filter_flag_df = True

RECORD_NUMBER = 3
steer_storage = np.zeros((RECORD_NUMBER, 1))

ld_change = 0
"""
케이스1: 왼쪽/오른쪽 아무것도 검출 안됨
-> 이전 웨이포인트를 추종한다
케이스2: 뭐라도 검출됨
-> 선의 중심을 따라간다
"""


def ld_callback(input_rosmsg):
    global ld_change
    ld_change = input_rosmsg.data


def text_marker_maker(name, frame_id, id, text, position, color_r=0, color_g=0, color_b=0, scale=0.6):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, 1)
    marker.pose.orientation.w = 1.0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.pose.position = Point(position[0], position[0], 0)
    marker.scale.z = scale
    marker.text = text

    return marker


def arrow_marker_maker(name, frame_id, id, tail, head, color_r=0, color_g=0, color_b=0, scale_x=0.2, scale_y=0.4):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, 1)
    marker.pose.orientation.w = 1.0
    marker.type = Marker.ARROW
    marker.scale = Vector3(scale_x, scale_y, 0)
    marker.points.append(Point(tail[0], tail[1], 0))
    marker.points.append(Point(head[0], head[1], 0))
    return marker


def pure_pursuit_point(point):
    global heading
    g_dis = np.hypot(point[0] + back_wheel, point[1])
    alpha = math.atan2(point[1], point[0] + back_wheel) - heading
    delta = math.atan2(2.0 * WB * math.sin(alpha) / g_dis, 1.0)
    # print("추종점 : ",'[',round(float(point[0]),2),",",round(float(point[1]),2),']')
    # print("pp_steer : ",round(float(math.degrees(delta)),2),"도")
    # print("-----------------------------------")
    return delta


def marker_array_rviz(markers):
    """make a MarkerArray object (여러 개의 Markers를 하나의 MarkerArray로 만들어 반환)

    Args:
        markers (list) : list of Marker objects. [marker, marker, ...]

    Returns:
        MarkerArray : MarkerArray object having input markers
    """
    marker_array = MarkerArray()
    for marker in markers:
        marker_array.markers.append(marker)

    return marker_array


def list_marker_maker(name, frame_id, id, list, color_r, color_g, color_b, scale):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = name
    marker.id = id
    marker.action = Marker.ADD
    marker.color = ColorRGBA(color_r / 255.0, color_g / 255.0, color_b / 255.0, 1)
    marker.pose.orientation.w = 1.0
    marker.type = Marker.POINTS
    marker.scale = Vector3(scale, scale, 0)
    for p in list:
        marker.points.append(Point(p[0], p[1], 0))
    return marker


def pcmaker(point_list, frame_id):
    pc = PointCloud()
    pc.header.frame_id = frame_id
    for p in point_list:
        point = Point32()
        point.x = p[0]
        point.y = p[1]
        point.z = 0
        pc.points.append(point)
    return pc


def linear(line, offset):
    new_line = []
    if len(line) > 1:
        line = distance_sort(line)
        total_line_len = 0
        for i in range(len(line) - 1):
            x_to_make = [line[i][0], line[i + 1][0]]
            y_to_make = [line[i][1], line[i + 1][1]]
            linear_line_fuc = interp.interp1d(x_to_make, y_to_make, kind='linear')
            piece_of_line_len = distance(line[i], line[i + 1])
            total_line_len = total_line_len + piece_of_line_len
            num = int(distance(line[i], line[i + 1]) // offset)
            if num == 0:
                num = 1
            x_offset = abs(x_to_make[-1] - x_to_make[0]) / num
            if x_to_make[0] < x_to_make[1]:
                x_new = [x_to_make[0]]
                for i in range(num - 1):
                    x_new.append(x_new[-1] + x_offset)
                for x in x_new:
                    y = float(linear_line_fuc(x))
                    new_line.append([x, y])
            else:
                x_new = [x_to_make[1]]
                for i in range(num - 1):
                    x_new.append(x_new[-1] + x_offset)
                for x in x_new:
                    y = float(linear_line_fuc(x))
                    new_line.append([x, y])

        return new_line, total_line_len

    else:
        return line, 0


def line_ruler(line):
    if len(line) > 1:
        total_dis = 0
        for i in range(len(line) - 1):
            dis = distance(line[i], line[i + 1])
            total_dis = total_dis + dis
        return total_dis
    else:
        return 0


def center_line_maker(line1, line2, line1_len, line2_len):
    '''
    기본적으론 짧은 차선 베이스로 중앙선을 뽑지만 중앙선이 ld보다 짧을 경우 긴 차선 베이스로 뽑는다
    '''
    center_line = [[-0.1, 0]]

    if line1_len > line2_len:
        short_line = line2
        long_line = line1

    else:
        short_line = line1
        long_line = line2

    main_line = short_line
    sub_line = long_line

    list1 = np.array(main_line)  # Example list1
    list2 = np.array(sub_line)  # Example list2

    # 두 점 간의 거리 행렬 계산
    distances = cdist(list1, list2)

    # 각 점에서 가장 가까운 점의 인덱스 찾기
    nearest_indices = np.argmin(distances, axis=1)

    # 대응 관계 구성
    for i, point1 in enumerate(list1):
        point2 = list2[nearest_indices[i]]
        center_line.append([(point1[0] * 0.5 + point2[0] * 0.5), (point1[1] * 0.5 + point2[1] * 0.5)])

    if line_ruler(center_line) < 100000:
        center_line = [[-0.1, 0]]
        list1 = np.array(sub_line)  # Example list1
        list2 = np.array(main_line)  # Example list2

        # 두 점 간의 거리 행렬 계산
        distances = cdist(list1, list2)

        # 각 점에서 가장 가까운 점의 인덱스 찾기
        nearest_indices = np.argmin(distances, axis=1)

        # 대응 관계 구성
        for i, point1 in enumerate(list1):
            point2 = list2[nearest_indices[i]]
            if distance(point1,point2)<3.5:
                center_line.append([(point1[0] * 0.5 + point2[0] * 0.5), (point1[1] * 0.5 + point2[1] * 0.5)])

    return center_line


def follower_choice(center_line):
    global look_distance
    global ex_follow_point

    # def find_nearest_point(point_list):
    #     distances = np.linalg.norm(point_list, axis=1)  # (0, 0)으로부터의 거리 계산
    # #     closest_index = np.argmin(np.abs(distances - look_distance))  # 거리가 2와 가장 가까운 인덱스 찾기
    # #     return point_list[closest_index]

    # follow_point=find_nearest_point(center_line)
    ld_flag = False
    for p in center_line:
        if distance([0, 0], p) > look_distance:
            follow_point = p
            ld_flag = True
            break

    if ld_flag is False:
        follow_point = center_line[-1]

    ex_follow_point = follow_point

    return follow_point


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def fit_curve_ransac(point_list, num_iterations, threshold_distance):
    global out_point_ransac
    out_point_ransac = []
    if filter_flag_r is False:
        return point_list
    else:
        if len(point_list) > 3:
            best_model = None
            best_inliers = []
            x = []
            y = []

            for p in point_list:
                x.append(p[0])
                y.append(p[1])

            for _ in range(num_iterations):
                # 랜덤하게 3개의 점 선택
                sample_indices = random.sample(range(len(x)), 3)
                sample_x = [x[i] for i in sample_indices]
                sample_y = [y[i] for i in sample_indices]

                # 2차 곡선 모델 계수 계산
                model_coefficients = np.polyfit(sample_x, sample_y, 2)

                # 나머지 점들과의 거리 계산
                distances = np.abs(np.polyval(model_coefficients, x) - y)

                # 임계값 이하의 거리를 가지는 inlier 점들 선택
                inlier_indices = np.where(distances < threshold_distance)[0]
                outlier_indices = np.where(distances > threshold_distance)[0]
                inliers = [(x[i], y[i]) for i in inlier_indices]
                outliers = [(x[i], y[i]) for i in outlier_indices]

                # 현재 모델이 이전보다 더 많은 inlier를 가지면 최적 모델 갱신
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_model = model_coefficients
                    best_outliers = outliers

            for p in best_outliers:
                out_point_ransac.append(p)

            return best_inliers
        else:
            return point_list


def distance_filter_for_left(point_list):
    global out_point_dis
    if filter_flag_d is False:
        return point_list
    else:
        if len(point_list) > 1:
            points_to_remove = []  # 삭제할 점들을 저장할 리스트

            for i, point1 in enumerate(point_list):
                for j, point2 in enumerate(point_list):
                    if (i != j) and distance(point1, point2) < min_dis:
                        if point1[1] < point2[1]:
                            points_to_remove.append(tuple(point2))
                        else:
                            points_to_remove.append(tuple(point1))

            # 중복된 점 제거
            unique_points_to_remove = list(set(points_to_remove))

            # point_list에서 삭제
            for point_to_remove in unique_points_to_remove:
                if point_to_remove in point_list:
                    point_list.remove(point_to_remove)
                    out_point_dis.append(point_to_remove)

            return point_list
        else:
            return point_list


def distance_filter_for_right(point_list):
    global out_point_dis
    if filter_flag_d is False:
        return point_list
    else:
        if len(point_list) > 1:
            points_to_remove = []  # 삭제할 점들을 저장할 리스트

            for i, point1 in enumerate(point_list):
                for j, point2 in enumerate(point_list):
                    if (i != j) and distance(point1, point2) < min_dis:
                        if point1[1] > point2[1]:
                            points_to_remove.append(tuple(point2))
                        else:
                            points_to_remove.append(tuple(point1))

            # 중복된 점 제거
            unique_points_to_remove = list(set(points_to_remove))

            # point_list에서 삭제
            for point_to_remove in unique_points_to_remove:
                if point_to_remove in point_list:
                    point_list.remove(point_to_remove)
                    out_point_dis.append(point_to_remove)

            return point_list
        else:
            return point_list


def distance_filter_for_far_point(points):
    global out_point_dis
    if filter_flag_df is True:
        remove_flag = False
        line = distance_sort(points)
        remove_index = -1
        start_point = line[0]
        for i in range(len(line) - 1):
            if distance(line[i], line[i + 1]) > max_dis:
                remove_index = i
                remove_flag = True
        if remove_flag is True:
            line_gone = line[remove_index + 1:]
            for p in line_gone:
                out_point_dis.append(p)
            line = line[0:remove_index]
        if not line:
            line = [start_point]

        return line
    else:
        return points


def noise_check(colors, noises, noise_offset=1.5):
    global append_noise
    """노이즈 중 콘으로 추정되는 것들을 담음"""
    for cone in colors:
        for noise in noises:
            if distance(cone, noise) < noise_offset:
                colors.append(noise)
                append_noise.append(noise)
                noises.remove(noise)
    return colors


def distance_sort(points):
    points_to_use = copy.deepcopy(points)
    sorted_list = []
    points_to_use.sort(key=lambda x: x[0])
    sorted_list.append(points_to_use[0])
    points_to_use.remove(points_to_use[0])

    while points_to_use:
        shortest_dis = 1000
        for p in points_to_use:
            now_dis = distance(sorted_list[-1], p)
            if now_dis < shortest_dis:
                shortest_dis = now_dis
                shortest_p = p

        if shortest_dis > 100:
            pass
        else:
            sorted_list.append(shortest_p)
        points_to_use.remove(shortest_p)

    return sorted_list


def connect_hole(line, gap, offset):
    new_line = []
    line = distance_sort(line)
    if len(line) > 1:
        for i in range(len(line) - 1):
            if distance(line[i], line[i + 1]) > gap:
                x_to_make = [line[i][0], line[i + 1][0]]
                y_to_make = [line[i][1], line[i + 1][1]]
                linear_line_fuc = interp.interp1d(x_to_make, y_to_make, kind='linear')
                num = int(distance(line[i], line[i + 1]) // offset)
                if num == 0:
                    num = 1
                x_offset = abs(x_to_make[-1] - x_to_make[0]) / num
                if x_to_make[0] < x_to_make[1]:
                    x_new = [x_to_make[0]]
                    for i in range(num - 1):
                        x_new.append(x_new[-1] + x_offset)
                    for x in x_new:
                        y = float(linear_line_fuc(x))
                        new_line.append([x, y])
                else:
                    x_new = [x_to_make[1]]
                    for i in range(num - 1):
                        x_new.append(x_new[-1] + x_offset)
                    for x in x_new:
                        y = float(linear_line_fuc(x))
                        new_line.append([x, y])
            else:
                new_line.append(line[i])
        new_line.append(line[-1])

        return new_line


def curve_sum(line):
    if len(line) > 1:
        total_dis = 0
        total_curve = 0
        total_curve_ = 0
        for i in range(len(line) - 1):
            if line[i][0] > 7:
                break
            delta_x = abs(line[i][0] - line[i + 1][0])
            delta_y = abs(line[i][1] - line[i + 1][1])
            delta_y_ = line[i + 1][1] - line[i][1]
            dis = distance(line[i], line[i + 1])
            if delta_y != 0:
                curve = math.atan2(delta_y, delta_x) * dis
                curve_ = math.atan2(delta_y_, delta_x) * dis
                total_curve = curve + total_curve
                total_curve_ = curve_ + total_curve_
                total_dis = dis + total_dis
        return total_curve / total_dis, total_curve_ / total_dis
    else:
        return 0


import math


def calculate_slope(point1, point2):
    # 두 점 사이의 기울기 계산
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        # 기울기가 무한대인 경우
        return None
    slope = (y2 - y1) / (x2 - x1)
    return slope


def create_new_point(coords):
    if len(coords) < 2:
        return None

    # 마지막 두 점 선택
    last_point = coords[-1]
    second_last_point = coords[-2]

    # 기울기 계산
    slope = calculate_slope(second_last_point, last_point)

    if slope is None:
        # 기울기가 무한대인 경우, 새로운 점을 생성할 수 없음
        return None

    # 마지막 점에서 길이 2만큼 떨어진 새로운 점 계산
    x_new = last_point[0] + 2 / math.sqrt(1 + slope ** 2)
    y_new = last_point[1] + (2 * slope) / math.sqrt(1 + slope ** 2)

    new_point = [x_new, y_new]
    return new_point


def callback(input_rosmsg):
    global detect_flag
    global out_point_dis
    global out_point_ransac
    global append_noise
    global ex_dwa_steer
    global RECORD_NUMBER 
    global steer_storage

    start_time = time.time()
    blue_cones = []
    yellow_cones = []
    noises = []
    raw = []
    # 수신된 데이터 저장
    for p in input_rosmsg.points:
        raw.append([p.x, p.y])
        if p.z == 1:
            blue_cones.append([p.x, p.y])
        elif p.z == 2:
            yellow_cones.append([p.x, p.y])
        else:
            noises.append([p.x, p.y])
    real_noise = copy.deepcopy(noises)

    # 색깔 -> 위치로 전환
    left_cones = yellow_cones
    right_cones = blue_cones
    cone_to_print_l = copy.deepcopy(yellow_cones)
    cone_to_print_r = copy.deepcopy(blue_cones)

    left_cones.insert(0, [0, offset / 2])
    right_cones.insert(0, [0, -1 * offset / 2])
    left_cones = noise_check(left_cones, noises, noise_offset)
    right_cones = noise_check(right_cones, noises, noise_offset)
    final_left_cones = []
    final_right_cones = []

    # 케이스1: 검출이 되지 않았을 경우
    if len(left_cones) < check_num and len(right_cones) < check_num:
        detect_flag = False
        follow_point = ex_follow_point
    else:
        detect_flag = True

    # 케이스 2: 검출이 된 경우
    if len(left_cones) >= check_num or len(right_cones) >= check_num:
        # 거리, ransac 으로 노이즈 제거
        final_left_cones = distance_filter_for_far_point(
            fit_curve_ransac(distance_filter_for_left(left_cones), num_it, t_dis))
        final_right_cones = distance_filter_for_far_point(
            fit_curve_ransac(distance_filter_for_right(right_cones), num_it, t_dis))

    # curve_ = 0.0

    # 검출이 됐을 경우 추종점 구하기
    if detect_flag is True:
        # 선분화
        right_line, right_line_len = linear(final_right_cones, line_offset)
        left_line, left_line_len = linear(final_left_cones, line_offset)

        # 중점 라인 생성
        center_line = center_line_maker(left_line, right_line, left_line_len, right_line_len)
        if line_ruler(center_line) < 3.5:
            center_line.append(create_new_point(center_line))
        center_line = connect_hole(center_line, 0.6, line_offset)
        curve_abs, curve_ = curve_sum(center_line)
        curve_abs = int(math.degrees(curve_abs))
        curve_ = int(math.degrees(curve_))
        center_line = tuple(map(lambda x: tuple(x), center_line))
        center_line=list(set(center_line))
        center_line=distance_sort(center_line)
        cur_abs_pub.publish(curve_abs)
        cur_pub.publish(curve_)        
        glob_path = list(map(lambda x: list(x), zip(*center_line)))
        global_path = GlobalPath(x=glob_path[0], y=glob_path[1])

        # 추종점 선택
        follow_point = follower_choice(center_line)

        line_sum = left_line + right_line + real_noise
        pc_line = pcmaker(line_sum, 'map')
        line_sum_pub.publish(pc_line)

        # # test
        dwa_steer = PT.gps_tracking_track(obs_xy=line_sum, path=global_path)

        ex_dwa_steer = dwa_steer
        # #

    if detect_flag is False:
        left_line = []
        right_line = []
        center_line = []
        dwa_steer = ex_dwa_steer

    detect_flag = False

    # steer뽑기
    pp_steer = math.degrees(pure_pursuit_point(follow_point)) * 71
    if pp_steer > 2000:
        pp_steer = 2000
    elif pp_steer < -2000:
        pp_steer = -2000
    # print("way len:",round(center_line_len,2))
    print("curve: ", curve_)
    print("curve_abs: ", curve_abs)
    print("dwa steer: ", round(float(-dwa_steer), 2))
    print("pp steer: ", round(float(pp_steer), 2))

    # if 50 > curve_ >= 40 or -50 < curve_ <= -40:
    #     steer = int(pp_steer * 0.4) + int(0.6 * -dwa_steer)
    # elif 50 <= curve_ or curve_ <= -50:
    #     steer = int(pp_steer * 0.2) + int(0.9 * -dwa_steer)
    # elif 40 > curve_ >= 30 or -40 < curve_ <= -30:
    #     steer = int(pp_steer * 0.5) + int(0.5 * -dwa_steer)
    # else:
    #     steer = int(pp_steer * 0.4) + int(0.5 * -dwa_steer)

    # Test ver 1
    # if curve_abs < 30:
    #     steer = int(0.4 * pp_steer + 0.5 * -dwa_steer)
    # elif 55 <= curve_abs:
    #     steer = int(0.2 * pp_steer + 1.0 * -dwa_steer)
    # else:
    #     steer = int((3e-05 * curve_ ** 3 - 0.0036 * curve_ ** 2 + 0.1272 * curve_ - 1.0) * pp_steer + \
    #                 (-3.5e-05 * curve_ ** 3 + 0.004375 * curve_ ** 2 - 0.1541 * curve_ + 2.166) * -dwa_steer)
    
    # Test ver 4
    if curve_abs < 35:
        steer = int(0.4 * pp_steer + 0.5 * -dwa_steer)
    elif 50 <= curve_abs:
        steer = int(0.2 * pp_steer + 0.9 * -dwa_steer)
        # steer = steer * 1.3
    else:
        steer = int((6.667e-06 * curve_ ** 3 - 0.0009 * curve_ ** 2 + 0.03183 * curve_ + 0.0625) * pp_steer + \
                    (-1.167e-05 * curve_ ** 3 + 0.001875 * curve_ ** 2 - 0.07471 * curve_ + 1.378) * -dwa_steer)
        # steer = steer * 1.3
    
    # if curve_abs - abs(curve_) > 10:
    #     steer *= 1.3
    # if pp_steer * -dwa_steer / 71 / 71 > 100:
    #      steer *= 1.3
    # steer = int(pp_steer * 0.4) + int(0.6 * -dwa_steer)

    for i in range(RECORD_NUMBER - 1, -1, -1) :
        steer_storage[i][0] = steer_storage[i-1][0]
    
    steer_storage[0][0] = steer
    
    if steer_storage[RECORD_NUMBER - 1][0] != 0:
        avg_steer = int(np.mean(steer_storage, axis=0)[0])
        
        if avg_steer - 500 < steer < avg_steer + 500:
            pass
        else:
            steer = steer_storage[0][0]
    
    print("통합 steer: ", round(float(steer), 2))

    steer_pub.publish(int(steer))
    steer2_pub.publish(int(pp_steer))
    text_to_pub = "steer:" + str(round(steer / 71, 2))

    # 데이터 시각화
    noise_data = list_marker_maker("noise", "map", 0, noises, 255, 255, 255, 0.3)
    left = list_marker_maker("left", "map", 1, left_line, 255, 255, 0, 0.1)
    right = list_marker_maker("right", "map", 2, right_line, 0, 0, 255, 0.1)
    center = list_marker_maker("center", "map", 3, center_line, 255, 255, 255, 0.1)
    arrow = arrow_marker_maker("leader", 'map', 4, tail=[0, 0], head=follow_point, color_r=255)
    cone_y = list_marker_maker("cone_y", 'map', 5, cone_to_print_l, 255, 255, 0, 0.5)
    cone_b = list_marker_maker("cone_b", 'map', 6, cone_to_print_r, 0, 0, 255, 0.5)
    outlier_dis = list_marker_maker("outlier_d", "map", 8, out_point_dis, 255, 0, 255, 0.5)
    outlier_ransac = list_marker_maker("outlier_r", "map", 9, out_point_ransac, 255, 0, 0, 0.5)
    text = text_marker_maker("steer_text", 'map', 7, text_to_pub, [-0.6, 2], 0, 255, 0)
    appended_noise = list_marker_maker("ap_noise", 'map', 10, append_noise, 244, 22, 74, 0.5)
    marker_array = marker_array_rviz(
        [left, right, center, arrow, cone_y, cone_b, text, outlier_dis, outlier_ransac, noise_data, appended_noise])
    marker_array_pub.publish(marker_array)

    append_noise = []
    out_point_ransac = []
    out_point_dis = []

    end_time = time.time()
    print('fps:', round(end_time - start_time, 2))
    print("=====================")


rospy.init_node('mission_track_sh', anonymous=True)
cone_sub = rospy.Subscriber('/cone', PointCloud, callback=callback, queue_size=1)
ld_sub = rospy.Subscriber('/look_distance', Float32, callback=ld_callback, queue_size=1)
line_sum_pub = rospy.Publisher('/line_sum', PointCloud, queue_size=1)
marker_array_pub = rospy.Publisher('/track_rviz', MarkerArray, queue_size=1)
steer_pub = rospy.Publisher('/track_steer', Int16, queue_size=1)
steer2_pub = rospy.Publisher('/track_steer2', Int16, queue_size=1)
cur_pub = rospy.Publisher('/cuv', Int16, queue_size=1)
cur_abs_pub = rospy.Publisher('/cuv_abs', Int16, queue_size=1)

rospy.spin()