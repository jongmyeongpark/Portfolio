#!/usr/bin/env python
# -- coding: utf-8 --

from matplotlib import offsetbox
import matplotlib as mpl
from math import cos, sin, pi
import numpy as np
import os, sys
import matplotlib.pyplot as plt
from find_s import find_s
from scipy.ndimage import rotate
from math import pi

#지도 정보 경로 설정
PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))) #/home/gigi/catkin_ws/src/macaron_3/
MAP="/path/npy_file/global_map/"
GBPATH="/path/npy_file/path/"


###########코드설명###########
#따놓은 전역경로 이름을 먼저 넣고, 실행을 시키면 k-city 맵과 내가 넣은 전역경로 플롯이 된다.
#그래프에 아무 영역이나 마우스로 클릭을 하고, 엔터키를 치면 해당 영역의 s좌표를 보여준다.
#전역경로 색상은 violet색상임. 
############################

########global_path#########
gp = "first_kcity_y_0728_all.npy"
############################

find_gp_s = find_s(gp)

#지도의 파일 개수. HD 맵의 차선을 추가하거나하면 수정해야함
line=27
center=21
bus=8

PATH=[]

# 같은 플롯 창에 모든 NPY 파일들 표시
plt.figure(figsize=(10, 10))
# y 축 라벨의 형식을 일반적인 실수 형태로 설정
# 좌표값의 지수 표기 비활성화
mpl.rcParams['axes.formatter.useoffset'] = False

# 소수점 이하 1자리까지 표시하도록 설정
plt.rcParams['axes.formatter.limits'] = (-1, 1)
print("draw k-city map")

# 이미지 읽어오기
image_path = '/home/macaron/catkin_ws/src/macaron_5/src/etc/kcity_map.png'
img = plt.imread(image_path)

# 초기 extent 설정
scale2 = 825
x_min = 935065.28
y_min = 1915634.53
angle = -0.8

img = rotate(img, angle/pi)
previous_img = plt.imshow(img, extent=[x_min, x_min + scale2+75, y_min, y_min + scale2], aspect='auto', alpha=1)

###################이미지 위치, 스케일, 회전 조정시 사용#####################
# # 이미지 위치 조정과 확대/축소를 위한 변수
# scale = 3.0

# def update_extent():
#     global x_min, y_min, scale, previous_img, angle, img
#     if previous_img is not None:
#         previous_img.remove()  # 이전 이미지 삭제
#     # 이미지 회전
#     previous_img = plt.imshow(img, extent=[x_min, x_min + scale2+75, y_min, y_min + scale2], aspect='auto', alpha=0.7)
#     plt.draw()

# def on_key(event):
#     global x_min, y_min, scale, scale2, angle

#     if event.key == 'up':
#         y_min -= 0.1 * scale
#         update_extent()
#     elif event.key == 'down':
#         y_min += 0.1 * scale
#         update_extent()
#     elif event.key == 'left':
#         x_min += 0.1 * scale
#         update_extent()
#     elif event.key == 'right':
#         x_min -= 0.1 * scale
#         update_extent()
#     elif event.key == '0':
#         scale2 += 10
#         update_extent()
#     elif event.key == '1':
#         scale2 -= 10
#         update_extent()
#     elif event.key == '8':
#         angle = 0.5
#         img = rotate(img, angle/pi)

#         update_extent()
#     elif event.key == '9':
#         angle = 0.5
#         img = rotate(img, angle/pi)

#         update_extent()

#     elif event.key == 'enter':
#         print(f'[x_min, y_min, scale2]: [{x_min :.2f} {y_min :.2f}, {scale2:.2f}], {angle:.2f}]')

# cid = plt.gcf().canvas.mpl_connect('key_press_event', on_key)
#######################################################################


for i in range(line):
    file_name="kcity_line_%d.npy"%(i+1)
    PATH.append(file_name)

for file_name in PATH:
    file_path = os.path.join(PATH_ROOT+MAP, file_name)
    #file_path = os.path.join(PATH_ROOT, file_path)
    print(file_path)
    try:
        data = np.load(file_path)
        plt.plot(data[:, 0], data[:, 1], "w", linewidth = 1)
    except FileNotFoundError:
        print(f"File not found: {file_path}")
PATH=[]


for i in range(center):
    file_name="kcity_center_%d.npy"%(i+line+1)
    PATH.append(file_name)

for file_name in PATH:
    file_path = os.path.join(PATH_ROOT+MAP, file_name)
    #file_path = os.path.join(PATH_ROOT, file_path)
    print(file_path)
    try:
        data = np.load(file_path)
        plt.plot(data[:, 0], data[:, 1], "w:", linewidth = 1)
    except FileNotFoundError:
        print(f"File not found: {file_path}")
PATH=[]


for i in range(bus):
    file_name="kcity_bus_%d.npy"%(i+line+center+1)
    PATH.append(file_name)
PATH.append("kcity_bus_static.npy")

for file_name in PATH:
    file_path = os.path.join(PATH_ROOT+MAP, file_name)
    #file_path = os.path.join(PATH_ROOT, file_path)
    print(file_path)
    try:
        data = np.load(file_path)
        plt.plot(data[:, 0], data[:, 1], "b")
    except FileNotFoundError:
        print(f"File not found: {file_path}")
PATH=[]


file_path = os.path.join(PATH_ROOT+GBPATH, gp)
data = np.load(file_path)

plt.plot(data[:, 0], data[:, 1], "violet", linewidth = 2, label=file_name)

##traffic_example##
coor = np.array([[935572.6787177685, 1915921.3513482576]])
plt.plot(coor[:, 0], coor[:, 1], "r", marker='o', markersize=4)

last_click = []

def on_click(event):
    global last_click
    if event.inaxes is not None:  # 그래프 영역을 클릭했을 때
        last_click = [event.xdata, event.ydata]

    def on_key(event):
        if event.key == 'enter':
            result = find_gp_s.main([last_click]) 
            print("x,y : ",last_click)
            plt.plot(last_click[0], last_click[1], 'ro', markersize=4)  # 클릭한 좌표에 빨간색 점으로 표시
            plt.text(last_click[0], last_click[1], f' s : {result:.2f}', fontsize=10, color='black')  # 결과값을 텍스트로 그래프에 표시
            plt.draw() 
            plt.gcf().canvas.mpl_disconnect(cid2)  # 키보드 이벤트 연결 해제
        else:
            pass

    cid2 = plt.gcf().canvas.mpl_connect('key_press_event', on_key) 

# 마우스 클릭 이벤트 연결
plt.gcf().canvas.mpl_connect('button_press_event', on_click)

# 좌표값 포맷 함수 정의
def format_coord(x, y):
    return f'x={x:.2f}, y={y:.2f}'

plt.xlabel("X")
plt.ylabel("Y")
plt.title("K-City Map")
plt.legend()
plt.grid()
# x와 y 축의 비율을 같게 설정
plt.axis('equal')

plt.gca().format_coord = format_coord  # 좌표값 출력 포맷 함수 설정

plt.show()