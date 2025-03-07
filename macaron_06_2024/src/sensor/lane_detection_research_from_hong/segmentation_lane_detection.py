#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import torch
import cv2
import sys
import numpy as np
import os
import warnings

from enum import IntEnum
from typing import Tuple
from ultralytics import YOLO
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

from std_msgs.msg import Float64, Bool

'''
조향 값
음수: 왼쪽으로 꺾음
양수: 오른쪽으로 꺾음
'''

current_dir = os.path.dirname(os.path.abspath(__file__))
VIDEO_PATH = r"/home/hong/KakaoTalk_20241006_193638023.mp4"
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print("Error: Could not open video.")
    sys.exit()

PT_PATH = os.path.join(current_dir, 'pt', 'toolgate_lane.pt')
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
MODEL = YOLO(PT_PATH).to(DEVICE)

prev_left_lane = np.array([])
prev_right_lane = np.array([])
prev_steer_angle = None
both_detected_info = None


class lane_shape_info(IntEnum):
    '''문자열을 숫자로 비교하기 위함입니다.
    이는 실행 속도를 향상시킵니다.
    '''
    Curve = 1
    Straight = 2

class steer_info(IntEnum):
    '''위와 똑같은 이유로 썼습니다.
    '''
    Straight = 1
    Left = 2
    Right = 3
    Safe = 4

def define_world_coordinate(frame: np.ndarray, map_height: np.ndarray, map_width: np.ndarray)-> np.ndarray:
    '''BEV의 frame 생성 (URL : https://gaussian37.github.io/vision-concept-ipm/)

    x : height
    y : width

    Args
        frame : 원본 frame
    
    Returns
        BEV Frame
    '''
    lut_frame = cv2.remap(frame, map_height, map_width, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    
    return lut_frame

def extract_mask_regression(mask: np.ndarray)-> Tuple[np.ndarray, np.ndarray]:
    '''차선에 segmentation(색칠 된?)곳에 회귀를 진행하였다.
    Args
        masks : 차선 segmentation된 정보들
    
    Returns
        x, y_fit : width, height로 회귀 정보들
    '''
    x = mask[:, 0].reshape(-1, 1)
    y = mask[:, 1]

    poly = PolynomialFeatures(degree=2)
    x_poly = poly.fit_transform(x)

    regress_model = LinearRegression()
    regress_model.fit(x_poly, y)

    x_min, x_max = x.min(), x.max()
    x_new = np.linspace(x_min, x_max, 40).reshape(-1, 1)
    x_poly_new = poly.transform(x_new)  # 수정된 부분: poly.fit_transform -> poly.transform

    y_fit_new = regress_model.predict(x_poly_new)

    return x_new, y_fit_new

def calculate_radiusCurvature(coeffs: np.ndarray, specific_coord: np.int64)-> np.float64:
    '''곡률을 구하고 곡률 반경을 구합니다.
    "https://www.youtube.com/watch?v=2oQnljpQm4Y"를 참조하였습니다.

    Args
        coeffs : Ax^2 + Bx + C
        specifit_coord : 차선에서의 특정 1개의 좌표를 의미

    Return
        curvature : 차선의 곡률 반경
    '''
    A = coeffs[0]
    B = coeffs[1]

    first_order = 2 * A * specific_coord + B
    second_order = 2 * A

    curvature = np.abs(second_order) / (1 + first_order ** 2) ** (3 / 2)
    radius_curvature = 1 / curvature

    return radius_curvature

def draw_lanes(lane_idx: int, frame: np.ndarray, x_fit: np.ndarray, y_fit: np.ndarray)-> np.ndarray:
    '''차선에 대한 정보를 frame에 그립니다.
    lane_idx는 0이면 왼쪽, 1이면 오른쪽을 의미합니다.

    Args
        frame : overlay의 정보로 frame의 복사본입니다.
        x_fit : x의 회귀 정보
        y_fit : y의 회귀 정보

    Return
        frame : overlay에 차선에 대한 정보를 그린 frame입니다.
    '''
    
    left_color, right_color = (255, 0, 0), (0, 0, 255)
    radius, thickness = 2, -1
    if lane_idx == 0:
        for (x, y) in zip(x_fit.flatten(), y_fit.flatten()):
            cv2.circle(frame, (int(x), int(y)), radius, left_color, thickness)
    elif lane_idx == 1:
        for (x, y) in zip(x_fit.flatten(), y_fit.flatten()):
            cv2.circle(frame, (int(x), int(y)), radius, right_color, thickness)

    return frame

def draw_center_lanes(frame: np.ndarray, concat_left_lane: np.ndarray, concat_right_lane: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''왼쪽과 오른쪽 차선의 점들을 받아 중앙차선을 그립니다.
    Args
        left_lane : 왼쪽 차선
        right_lane : 오른쪽 차선
    
    Return
        frame : 중앙 차선이 그려진 frame

    '''
    center_lane = []
    center_color = (0, 255, 0)
    radius, thickness = 2, -1

    sorted_concat_left_lane = sorted(concat_left_lane, key=lambda x: x[1], reverse=True)
    sorted_concat_right_lane = sorted(concat_right_lane, key=lambda x: x[1], reverse=True)
    
    for (lx, ly), (rx, ry) in zip(sorted_concat_left_lane, sorted_concat_right_lane):
        center_x = int((lx + rx) / 2)
        center_y = int((ly + ry) / 2)
        center = [center_x, center_y]
        center_lane.append(center)
        cv2.circle(frame, (center_x, center_y), radius, center_color, thickness)

    return frame, np.array(center_lane)

def adjust_vehicle_direction(frame: np.ndarray, center_lane: np.ndarray)-> Tuple[np.ndarray, IntEnum]:
    '''ERP-42가 바라보는 곳과 중앙선의 좌표 추종으로 steer를 꺾을 수 있게 만듭니다.
    center_lane은 [width, height] 정보를 받아옵니다.
    Arg
        frame: 왼, 오, 중앙선의 차선을 모두 그린 frame
        center_lane : 중앙 차선을 의미합니다.
    
    Return
        frame: 차량이 바라보고 있는 곳을 frame에 그립니다.
        steer_direction: steer를 왼쪽, 오른쪽으로 꺾을지를 반환합니다.
    '''
    vehicle_direction_height_max = int(frame.shape[0])
    vehicle_direction_height_min = int(frame.shape[0] // 2) + 10
    vehicle_direction_width = int(frame.shape[1] // 2)
    
    vehicle_top = (vehicle_direction_width, vehicle_direction_height_min)
    vehicle_bottom = (vehicle_direction_width, vehicle_direction_height_max)

    steer_direction = steer_info.Straight
    if np.any(center_lane):
        center_lane_width_critertion = center_lane[:, 0][-1]
        if center_lane_width_critertion > vehicle_direction_width:
            steer_direction = steer_info.Right # 3
        elif center_lane_width_critertion < vehicle_direction_width:
            steer_direction = steer_info.Left # 2

    cv2.line(frame, vehicle_top, vehicle_bottom, (0, 255, 255), 2)

    return frame, steer_direction

def angle_of_steer(center_lane: np.ndarray, left_lane: np.ndarray, right_lane: np.ndarray, is_right_overlap: bool, is_left_overlap: bool)-> np.float64:
    '''중앙선과 ERP-42가 바라보는 각도를 구해줍니다.
    이는 cos(θ) = (a * b) / (|a| |b|)로 각각의 방향벡터로 구할 수 있습니다.
    
    Args
        center_lane : 중앙선을 의미합니다.

    Return
        angle: 중앙선과 ERP-42가 바라보는 각도 값을 반환합니다.

    '''
    erp_direction_unit_vector = np.array([0, -1])  # a

    # 두 차선이 다 잘 그려지고, 중앙선까지 잘 구해졌을 때
    if not is_right_overlap and not is_left_overlap:
        if center_lane is None or len(center_lane) <= 1:
            return 0

        center_lane_direction_vector = np.array([center_lane[-1][0] - center_lane[0][0], center_lane[-1][1] - center_lane[0][1]]) # b
        norm_center_lane = np.linalg.norm(center_lane_direction_vector)
        
        if norm_center_lane == 0:
            return 0

        center_lane_direction_unit_vector = center_lane_direction_vector / norm_center_lane  

        dot_product = np.dot(erp_direction_unit_vector, center_lane_direction_unit_vector)
        dot_product = np.clip(dot_product, -1.0, 1.0) # arccos은 -1 ~ 1의 치역을 가지고 있음.
        angle_rad = np.arccos(dot_product)

        angle_degree = np.rad2deg(angle_rad)
        angle_degree = np.clip(angle_degree, -20, 20)

        return angle_degree

    # 왼/오른쪽 차선이 겹쳐, 오른쪽 차선과 heading의 조향 값 추출    
    elif is_right_overlap and not is_left_overlap:
        if right_lane is None or len(right_lane) <= 1:
            return 0
        
        right_lane_direction_vector = np.array([right_lane[-1][0] - right_lane[0][0], right_lane[-1][1] - right_lane[0][1]])
        norm_right_lane = np.linalg.norm(right_lane_direction_vector)

        if norm_right_lane == 0:
            return 0
        
        right_lane_direction_unit_vector = right_lane_direction_vector / norm_right_lane

        dot_product = np.dot(erp_direction_unit_vector, right_lane_direction_unit_vector)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle_rad = np.arccos(dot_product)

        angle_degree = np.rad2deg(angle_rad)
        angle_degree = np.clip(angle_degree, -10, 10)

        return angle_degree
    
    elif not is_right_overlap and is_left_overlap:
        if left_lane is None or len(left_lane) <= 1:
            return 0
        
        left_lane_direction_vector = np.array([left_lane[-1][0] - left_lane[0][0], left_lane[-1][1] - left_lane[0][1]])
        norm_left_lane = np.linalg.norm(left_lane_direction_vector)

        if norm_left_lane == 0:
            return 0
        
        left_lane_direction_unit_vector = left_lane_direction_vector / norm_left_lane

        dot_product = np.dot(erp_direction_unit_vector, left_lane_direction_unit_vector)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle_rad = np.arccos(dot_product)

        angle_degree = np.rad2deg(angle_rad)
        angle_degree = np.clip(angle_degree, -10, 10)

        return angle_degree


def exception_extract_centerLane_of_bothLanes(frame: np.ndarray, is_right_overlap: bool, is_left_overlap: bool)-> Tuple[float, float]:
    '''왼/오른쪽의 차선을 잘 인지하여도, 중앙선과 헤딩의 방향벡터가 평행에 가깝다면 중앙선으로 경로를 추종하지 못하게 됩니다.
    따라서, heading과 왼쪽/오른쪽 차선과의 유클리디안 거리를 구하게 됩니다.
    
    standard_lane_area는 일반적인 도로 폭 값입니다.
    
    Args:
        bev_frame: ipm으로 만든 이미지입니다.
    
    Returns:
        additional_steer_info: 왼쪽(Left)으로 갈지, 오른쪽(Right)으로 갈지에 대한 정보입니다. 만약 조건문안에 들지 않는다면 None을 반환하는 것에 주의하세요.
    '''
    standard_lane_area = 325 # cm
    real_left_to_heading_distance, real_right_to_heading_distance = 0, 0
    
    left_lane = cv2.inRange(frame, (255, 0, 0), (255, 0, 0))
    right_lane = cv2.inRange(frame, (0, 0, 255), (0, 0, 255))
    heading_lane = cv2.inRange(frame, (0, 255, 255), (0, 255, 255))
    
    left_lane_index = np.argwhere(left_lane > 0) # 내림 차순 (height, width)
    right_lane_index = np.argwhere(right_lane > 0)
    heading_lane_index = np.argwhere(heading_lane > 0)

    # 차선이 겹치지 않고, 잘 검출 했을 때
    if np.any(right_lane_index) and np.any(left_lane_index) and np.any(heading_lane_index) and not is_left_overlap and not is_right_overlap:
        left_width_mean = np.mean(left_lane_index[:, 1])
        right_width_mean = np.mean(right_lane_index[:, 1])
        heading_width_mean = np.mean(heading_lane_index[:, 1])

        left_to_heading_pixel_distance = abs(int(heading_width_mean - left_width_mean))
        right_to_heading_pixel_distance = abs(int(right_width_mean - heading_width_mean))
        left_to_right_heading_pixel_distance = abs(int(right_width_mean - left_width_mean))


        real_lane_area = standard_lane_area / left_to_right_heading_pixel_distance 
        real_left_to_heading_distance = left_to_heading_pixel_distance * real_lane_area
        real_right_to_heading_distance = right_to_heading_pixel_distance * real_lane_area

        return real_left_to_heading_distance, real_right_to_heading_distance
    
    # 차선이 오른쪽으로 겹치고, 오른쪽과의 거리만 추출할 때
    elif np.any(right_lane_index) and np.any(heading_lane_index) and not is_left_overlap and is_right_overlap:
        right_width_mean = np.mean(right_lane_index[:, 1])
        heading_width_mean = np.mean(heading_lane_index[:, 1])

        right_to_heading_pixel_distance = abs(int(right_width_mean - heading_width_mean))
        
        real_lane_area = standard_lane_area / 400 # 오른쪽 - 왼쪽 간의 차선 pixel간격은 300~500이므로 400
        real_right_to_heading_distance = right_to_heading_pixel_distance * real_lane_area
        
        return -1, real_right_to_heading_distance

    # 차선이 왼쪽으로 겹치고, 왼쪽과의 거리만 추출할 때
    elif np.any(left_lane_index) and np.any(heading_lane_index) and not is_right_overlap and is_left_overlap:
        left_width_mean = np.mean(left_lane_index[:, 1])
        heading_width_mean = np.mean(heading_lane_index[:, 1])

        left_to_heading_pixel_distance = abs(int(heading_width_mean - left_width_mean))

        real_lane_area = standard_lane_area / 400 
        real_left_to_heading_distance = left_to_heading_pixel_distance * real_lane_area

        return real_left_to_heading_distance, -1

    # 이상치,, 갑자기 1000나옴 이거 뭐냐 ㅅㅂ?
    if real_left_to_heading_distance > 325 or real_right_to_heading_distance > 325:
        return 160, 160

    return real_left_to_heading_distance, real_right_to_heading_distance

def confirm_overlap_lanes(left: np.ndarray, right: np.ndarray, threshold: int = 100) -> bool:
    ''' 차선이 겹치는지에 대한 유무
    Args:
        left: left lane, shape: (40, 2)
        right: right lane, ''

    Return:
        is_overlap: 겹치면 True
    '''
    is_overlap = False
    is_left_ovrelap = False
    is_right_overlap = False

    if np.any(left) and np.any(right):
        left_width_mean = np.mean(left[:, 0])
        right_width_mean = np.mean(right[:, 0])


        diff = abs(left_width_mean - right_width_mean)
        if diff <= threshold:
            is_overlap = True

            if is_overlap:
                if left_width_mean <= 320:
                    is_left_ovrelap = True

                if left_width_mean > 320:
                    is_right_overlap = True


    return is_overlap, is_left_ovrelap, is_right_overlap        
    
def verify_lanes_detect_video():
    while True:
        # 어떻게든 클래스를 쓰지 않겠다는 발악으로,, 어쩔 수 없이 global선언합니다.
        global prev_left_lane
        global prev_right_lane
        global prev_steer_angle
        global both_detected_info

        is_overlap = False
        is_right_overlap = False
        is_left_overlap = False

        ret, frame = cap.read()
        if not ret:
            print("No Frame in detect lines")
            exit()
        frame = cv2.resize(frame, (640, 480))

        overlay = frame.copy()
        overlap_frame = frame.copy()
        masks = []
        results = MODEL(frame, verbose=False)

        concat_left_lane = np.array([])
        concat_right_lane = np.array([])
        center_lane = np.array([])

        exception_concat_left_lane = np.empty((0, 2))
        exception_concat_right_lane = np.empty((0, 2))

        for result in results:
            if result.masks:
                masks = result.masks.xy
            
            detect_class_index = np.array(result.boxes.cls.tolist())
            detect_class_cnt = len(detect_class_index)
            
            if detect_class_cnt <=  2: 
                for mask in masks:
                    if np.any(mask):
                        x_fit, y_fit = extract_mask_regression(mask)
                        counts = sum(np.sum(x_fit < (frame.shape[1] // 2), axis=1))
                        expand_dim_y = y_fit.reshape(-1, 1)
                        if counts >= 10:
                            concat_left_lane = np.hstack((x_fit, expand_dim_y))
                            exception_concat_left_lane = np.vstack((exception_concat_left_lane, concat_left_lane))
                            if exception_concat_left_lane.size == 160:
                                right_x_fit, right_y_fit = exception_concat_left_lane[:40, 0], exception_concat_left_lane[:40, 1]
                                left_x_fit, left_y_fit = exception_concat_left_lane[40:, 0], exception_concat_left_lane[40:, 1]

                                concat_left_lane = np.array([])
                                concat_right_lane = np.array([])
                                
                                expand_dim_right_x, expand_dim_right_y = right_x_fit.reshape(-1, 1), right_y_fit.reshape(-1, 1)
                                expand_dim_left_x, expand_dim_left_y = left_x_fit.reshape(-1, 1), left_y_fit.reshape(-1, 1)
                                concat_left_lane = np.hstack((expand_dim_left_x, expand_dim_left_y))
                                concat_right_lane = np.hstack((expand_dim_right_x, expand_dim_right_y))

                                overlay = draw_lanes(0, overlay, left_x_fit, left_y_fit)
                                overlay = draw_lanes(1, overlay, right_x_fit, right_y_fit)
                            
                            else:  
                                overlay = draw_lanes(0, overlay, x_fit, y_fit)
                        else:
                            concat_right_lane = np.hstack((x_fit, expand_dim_y))
                            exception_concat_right_lane = np.vstack((exception_concat_right_lane, concat_right_lane))
                            if exception_concat_right_lane.size == 160:
                                left_x_fit, left_y_fit = exception_concat_right_lane[40:, 0], exception_concat_right_lane[40:, 1]
                                right_x_fit, right_y_fit = exception_concat_right_lane[:40, 0], exception_concat_right_lane[:40, 1]

                                concat_left_lane = np.array([])
                                concat_right_lane = np.array([])
                                
                                expand_dim_left_x, expand_dim_left_y = left_x_fit.reshape(-1, 1), left_y_fit.reshape(-1, 1)
                                expand_dim_right_x, expand_dim_right_y = right_x_fit.reshape(-1, 1), right_y_fit.reshape(-1, 1)
                                
                                concat_left_lane = np.hstack((expand_dim_left_x, expand_dim_left_y))
                                concat_right_lane = np.hstack((expand_dim_right_x, expand_dim_right_y))
                                
                                # 여기만 한정해서, index바꿔줌 left: 1, right: 0 (굳이 sorting을 사용해서 왼/오른쪽 나누지 않음 -> 연산량 신경씀)
                                overlay = draw_lanes(1, overlay, left_x_fit, left_y_fit)
                                overlay = draw_lanes(0, overlay, right_x_fit, right_y_fit)
                            
                            else:
                                overlay = draw_lanes(1, overlay, x_fit, y_fit)

                    is_overlap, is_left_overlap, is_right_overlap = confirm_overlap_lanes(concat_left_lane, concat_right_lane)

                    if np.any(concat_left_lane) and np.any(concat_left_lane) and is_overlap: # 차선이 겹칠 때
                        overlay = overlap_frame.copy()
                        if is_right_overlap:
                            overlay = draw_lanes(1, overlay, concat_right_lane[:, 0], concat_right_lane[:, 1])

                        elif is_left_overlap:
                            overlap = draw_lanes(0, overlap, concat_left_lane[:, 0], concat_right_lane[:, 1])                        

                    elif np.any(concat_left_lane) and np.any(concat_right_lane) and not is_overlap: # 차선이 겹치지 않을 때 
                        overlay, center_lane = draw_center_lanes(overlay, concat_left_lane, concat_right_lane)
                        prev_left_lane, prev_right_lane = concat_left_lane, concat_right_lane
                        both_detected_info = "Current Lanes: Detected"

        if detect_class_cnt != 2 and np.any(prev_left_lane) and np.any(prev_right_lane):
            overlay = draw_lanes(0, overlay, prev_left_lane[:, 0], prev_left_lane[:, 1])
            overlay = draw_lanes(1, overlay, prev_right_lane[:, 0], prev_right_lane[:, 1])
            overlay, center_lane = draw_center_lanes(overlay, prev_left_lane, prev_right_lane)
            both_detected_info = "Previous Lanes: No Detected"

        overlay, steer = adjust_vehicle_direction(overlay, center_lane)
        left2heading_distance, right2heading_distance = exception_extract_centerLane_of_bothLanes(overlay, is_right_overlap, is_left_overlap)
        current_steer_angle = angle_of_steer(center_lane, concat_left_lane, concat_right_lane, is_right_overlap, is_left_overlap)
        

        steer_text = None
        if steer == 1:
            steer_text = 'Straight'
        elif steer == 2:
            steer_text = 'Left'
            current_steer_angle = - current_steer_angle
        elif steer == 3:
            steer_text = 'Right'
        
        if prev_steer_angle is not None and current_steer_angle != prev_steer_angle:
            correct_thres = max(prev_steer_angle, current_steer_angle) - min(prev_steer_angle, current_steer_angle)
            if correct_thres >= 5:
                current_steer_angle = (prev_steer_angle + current_steer_angle) / 2

        # steer_angle_pub.publish(np.deg2rad(current_steer_angle)) # steer_angle: '-' if steer_angle < 0  else '+'
        # left_heading_distance_pub.publish(left2heading_distance)
        # right_heading_distance_pub.publish(right2heading_distance)
        # invade_lane_pub.publish(reverse_bool) # 옆 차선을 침범했는지 안했는지, 만약 했으면 True, 아니면 False

        prev_steer_angle = current_steer_angle
        alpha = 1       
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        cv2.putText(frame, f'Steer : {steer_text}', (0, 125), 1, 1, (0, 255, 0), 2)
        cv2.putText(frame, f'Steer Angle : {current_steer_angle}', (0, 145), 1, 1, (0, 0, 255), 2)
        cv2.putText(frame, f'Detected Lanes : {both_detected_info}', (0, 165), 1, 1, (0, 255, 255), 2)
        cv2.putText(frame, f'Left-Heading Distance: {left2heading_distance}', (0, 180), 1, 1, (0, 0, 255), 2)
        cv2.putText(frame, f'Right Heading Distance: {right2heading_distance}', (0, 195), 1, 1, (0, 0, 255), 2)

        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        # return np.deg2rad(current_steer_angle), left2heading_distance, right2heading_distance
       
    cap.release()
    cv2.destroyAllWindows()

def detect_lines():
    # 어떻게든 클래스를 쓰지 않겠다는 발악으로,, 어쩔 수 없이 global선언합니다.
    global prev_left_lane
    global prev_right_lane
    global prev_steer_angle
    global both_detected_info

    is_overlap = False
    is_right_overlap = False
    is_left_overlap = False

    ret, frame = cap.read()
    if not ret:
        print("No Frame in detect lines function")
        sys.exit()
    frame = cv2.resize(frame, (640, 480))

    overlay = frame.copy()
    overlap_frame = frame.copy()

    masks = []
    results = MODEL(frame, verbose=False)

    concat_left_lane = np.array([])
    concat_right_lane = np.array([])
    center_lane = np.array([])

    exception_concat_left_lane = np.empty((0, 2))
    exception_concat_right_lane = np.empty((0, 2))

    for result in results:
        if result.masks:
            masks = result.masks.xy
        
        detect_class_index = np.array(result.boxes.cls.tolist())
        detect_class_cnt = len(detect_class_index)
        
        if detect_class_cnt == 2: 
            for mask in masks:
                if np.any(mask):
                    x_fit, y_fit = extract_mask_regression(mask)
                    counts = sum(np.sum(x_fit < (frame.shape[1] // 2), axis=1))
                    expand_dim_y = y_fit.reshape(-1, 1)
                    if counts >= 10:
                        concat_left_lane = np.hstack((x_fit, expand_dim_y))
                        exception_concat_left_lane = np.vstack((exception_concat_left_lane, concat_left_lane))
                        if exception_concat_left_lane.size == 160:
                            right_x_fit, right_y_fit = exception_concat_left_lane[:40, 0], exception_concat_left_lane[:40, 1]
                            left_x_fit, left_y_fit = exception_concat_left_lane[40:, 0], exception_concat_left_lane[40:, 1]

                            concat_left_lane = np.array([])
                            concat_right_lane = np.array([])
                            
                            expand_dim_right_x, expand_dim_right_y = right_x_fit.reshape(-1, 1), right_y_fit.reshape(-1, 1)
                            expand_dim_left_x, expand_dim_left_y = left_x_fit.reshape(-1, 1), left_y_fit.reshape(-1, 1)
                            concat_left_lane = np.hstack((expand_dim_left_x, expand_dim_left_y))
                            concat_right_lane = np.hstack((expand_dim_right_x, expand_dim_right_y))

                            overlay = draw_lanes(0, overlay, left_x_fit, left_y_fit)
                            overlay = draw_lanes(1, overlay, right_x_fit, right_y_fit)
                        
                        else:  
                            overlay = draw_lanes(0, overlay, x_fit, y_fit)
                    else:
                        concat_right_lane = np.hstack((x_fit, expand_dim_y))
                        exception_concat_right_lane = np.vstack((exception_concat_right_lane, concat_right_lane))
                        if exception_concat_right_lane.size == 160:
                            left_x_fit, left_y_fit = exception_concat_right_lane[40:, 0], exception_concat_right_lane[40:, 1]
                            right_x_fit, right_y_fit = exception_concat_right_lane[:40, 0], exception_concat_right_lane[:40, 1]

                            concat_left_lane = np.array([])
                            concat_right_lane = np.array([])
                            
                            expand_dim_left_x, expand_dim_left_y = left_x_fit.reshape(-1, 1), left_y_fit.reshape(-1, 1)
                            expand_dim_right_x, expand_dim_right_y = right_x_fit.reshape(-1, 1), right_y_fit.reshape(-1, 1)
                            
                            concat_left_lane = np.hstack((expand_dim_left_x, expand_dim_left_y))
                            concat_right_lane = np.hstack((expand_dim_right_x, expand_dim_right_y))
                            
                            # 여기만 한정해서, index바꿔줌 left: 1, right: 0 (굳이 sorting을 사용해서 왼/오른쪽 나누지 않음 -> 연산량 신경씀)
                            overlay = draw_lanes(1, overlay, left_x_fit, left_y_fit)
                            overlay = draw_lanes(0, overlay, right_x_fit, right_y_fit)
                        
                        else:
                            overlay = draw_lanes(1, overlay, x_fit, y_fit)

                is_overlap, is_left_overlap, is_right_overlap = confirm_overlap_lanes(concat_left_lane, concat_right_lane)

                if np.any(concat_left_lane) and np.any(concat_left_lane) and is_overlap: # 차선이 겹칠 때
                    overlay = overlap_frame.copy()
                    if is_right_overlap:
                        overlay = draw_lanes(1, overlay, concat_right_lane[:, 0], concat_right_lane[:, 1])

                    elif is_left_overlap:
                        overlap = draw_lanes(0, overlap, concat_left_lane[:, 0], concat_right_lane[:, 1])                        

                elif np.any(concat_left_lane) and np.any(concat_right_lane) and not is_overlap: # 차선이 겹치지 않을 때 
                    overlay, center_lane = draw_center_lanes(overlay, concat_left_lane, concat_right_lane)
                    prev_left_lane, prev_right_lane = concat_left_lane, concat_right_lane
                    both_detected_info = "Current Lanes: Detected"
            
    if detect_class_cnt != 2 and np.any(prev_left_lane) and np.any(prev_right_lane):
        overlay = draw_lanes(0, overlay, prev_left_lane[:, 0], prev_left_lane[:, 1])
        overlay = draw_lanes(1, overlay, prev_right_lane[:, 0], prev_right_lane[:, 1])
        overlay, center_lane = draw_center_lanes(overlay, prev_left_lane, prev_right_lane)
        both_detected_info = "Previous Lanes: No Detected"
    
    overlay, steer = adjust_vehicle_direction(overlay, center_lane)
    left2heading_distance, right2heading_distance = exception_extract_centerLane_of_bothLanes(overlay, is_right_overlap, is_left_overlap)
    current_steer_angle = angle_of_steer(center_lane, concat_left_lane, concat_right_lane, is_right_overlap, is_left_overlap)
    
    steer_text = None

    if steer == 1:
        steer_text = 'Straight'
    elif steer == 2:
        steer_text = 'Left'
        current_steer_angle = - current_steer_angle
    elif steer == 3:
        steer_text = 'Right'
    
    if prev_steer_angle is not None and current_steer_angle != prev_steer_angle:
        correct_thres = max(prev_steer_angle, current_steer_angle) - min(prev_steer_angle, current_steer_angle)
        if correct_thres >= 5:
            current_steer_angle = (prev_steer_angle + current_steer_angle) / 2

    prev_steer_angle = current_steer_angle
    alpha = 1       
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    cv2.putText(frame, f'Steer : {steer_text}', (0, 125), 1, 1, (0, 255, 0), 2)
    cv2.putText(frame, f'Steer Angle : {current_steer_angle}', (0, 145), 1, 1, (0, 0, 255), 2)
    cv2.putText(frame, f'Detected Lanes : {both_detected_info}', (0, 165), 1, 1, (0, 255, 255), 2)
    cv2.putText(frame, f'Left-Heading Distance: {left2heading_distance}', (0, 180), 1, 1, (0, 0, 255), 2)
    cv2.putText(frame, f'Right Heading Distance: {right2heading_distance}', (0, 195), 1, 1, (0, 0, 255), 2)

    cv2.imshow('frame', frame)

    cv2.waitKey(1)

    return np.deg2rad(current_steer_angle), left2heading_distance, right2heading_distance
       
def main():
    rospy.init_node('lane_detection', anonymous=True)
    
    # detect_lines() # 희승이한테 보내주는 함수
    verify_lanes_detect_video() # 나 혼자 확인용(이 코드만 실행할 시)

if __name__ == "__main__":
    main()