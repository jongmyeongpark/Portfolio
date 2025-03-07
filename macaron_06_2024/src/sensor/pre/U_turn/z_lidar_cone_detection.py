#!/usr/bin/env python
# -- coding: utf-8 --

#morai-> lidar_cone_detection.py 참고

import rospy #ros와 통신을 위한 기능 제공
import time 
import math
import numpy as np #배열 및 행렬 연산 
from collections import defaultdict #키가 존재하지 않을 때 기본값을 생성하는 딕셔너리 
"""
 "키"란 딕셔너리에서 유니크한 식별자로 사용되는 객체입니다. Python의 딕셔너리는 키:값 쌍으로 데이터를 저장합니다. 
 각각의 키(key)는 딕셔너리 내에서 해당 키에 연결된 값(value)을 찾는데 사용됩니다.

defaultdict의 작동 방식
defaultdict를 사용할 때는, 생성자에 딕셔너리가 존재하지 않는 키에 대해 반환할 기본값을 생성하는 함수를 인자로 제공해야 합니다. 이 함수는 아무런 인자를 받지 않고, 호출될 때마다 새로운 기본값을 반환합니다.

예를 들어, 리스트의 기본값을 가진 defaultdict를 생성하려면 다음과 같이 작성할 수 있습니다.
예시
from collections import defaultdictf callback(self, input_rosmsg):
        
        obs_xyz=list(map(lambda x: list(x), pc2.read_points(input_rosmsg , skip_nans=True)))
        #cluster기준으로 정렬 // x[:2]=point x[3]=cluster x[4]=intensity
        obs_xyz.sort(key=lambda x:x[3])
        cone_centers=[]

        for cluster_num in range(obs_xyz[-1][3]+1):

            x_list=[]
            y_list=[]
            z_list=[]
            in_list=[]
            for p in obs_xyz:
                if cluster_num==p[3]:
                    x_list.append(p[0])
                    y_list.append(p[1])
                    z_list.append(p[2])
                    in_list.append(p[4])


d = defaultdict(list)
이 경우, d에 존재하지 않는 키에 접근하려고 하면, 자동으로 빈 리스트 []를 기본값으로 가지는 새로운 항목이 생성됩니다.

예시
from collections import defaultdict

d = defaultdict(list) # 빈 리스트를 기본값으로 설정
d['a'].append(1) # 'a' 키에 대한 값으로 1을 추가
d['a'].append(2) # 'a' 키에 대한 값으로 2를 추가
d['b'].append(4) # 'b' 키가 존재하지 않으므로 빈 리스트를 생성 후 4를 추가

print(d)
# 출력: defaultdict(<class 'list'>, {'a': [1, 2], 'b': [4]})
위 예시에서 볼 수 있듯이, defaultdict는 존재하지 않는 키에 대해 자동으로 빈 리스트를 생성하고, 
그 리스트에 값을 추가하는 과정을 간소화합니다. 
이러한 특성은 데이터를 그룹화하거나 다중 값을 가지는 딕셔너리를 쉽게 작성할 때 매우 유용합니다.
"""
from geometry_msgs.msg import Point32 
from sensor_msgs.msg import PointCloud, ChannelFloat32, Imu
#ROS에서 3D 점과 점군 데이터를 표현하는 메시지 타입 임포트, imu데이터 메시지 타입 임포트 

class Cone_detection:
    def __init__(self):
        self.lidar_sub=rospy.Subscriber('object3D',PointCloud,self.callback,queue_size=1)  
        #dbscan_result /velodyne_points ( object3D,PointCloud이게 원래야) 
        self.cone_pub=rospy.Publisher('cone',PointCloud,queue_size=1)

        self.cone_max_lenth=0.37 #m단위 콘 폭 0.37 높이 0.7
        self.cone_max_height=0.7

#input_rosmsg : pointcloud 데이터를 실시간으로 담는 변수 
    def callback(self, input_rosmsg): 
        label = [] #라벨을 저장할 빈 리스트 생성. 나중에 각 포인트에 할당됨 
        point = [[p.x, p.y, p.z] for p in input_rosmsg.points]
        #변수 p를 따로 선언 안하고 for루프 내에서만 유효하게 사용 가능
        #객체의 속성이나 매서드를 호출할 떼 . 연산자를 사용 (객체지향프로그래밍)
#.points는 PointCloud 메시지의 속성입니다.
# points 속성은 Point32 메시지 타입의 객체 리스트를 저장합니다. 
# input_rosmsg.points는 input_rosmsg 변수에 저장된 PointCloud 메시지의 points 속성을 의미
#input_rosmsg는 LiDAR로부터 받은 3D 점 군 데이터를 저장하는 변수입니다.
#.points는 input_rosmsg 변수에 저장된 3D 점 군 데이터의 각 점 정보를 리스트 형태로 제공합니다.

        
        for channel in input_rosmsg.channels:     #각 채널에 대해 반복 작업 수행  
            label = [c for c in channel.values] 
            #리스트 컴프리헨션으로 values값 추출해서 label 리스트 만듬 
            #[식 for 변수 in 반복가능한 객체 if 조건] >> 리스트 컴프리헨션 

            '''
            for a in b : 반복문 , b라는 객체(리스트,문자열,딕셔너리)의 각 항목을 a라는 변수에 차례대로 할당 
            
            # 리스트 반복
                my_list = ["a", "b", "c"]
                for item in my_list:
                    print(item)

                # 출력 결과:
                # a
                # b
                # c

                # 문자열 반복
                my_string = "Hello"
                for char in my_string:
                    print(char)

                # 출력 결과:
                # H
                # e
                # l
                # l
                # o
                            
            '''
        label_points = defaultdict(list)
        #collection 모듈에서 제공하는 defaultdict를 사용 
        #라벨별 포인트를 저장할 딕셔너리를 생성 => 이는 존재하지 않는 키에 대해 자동으로 빈 리스트를 기본 값으로 생성함 
        for l, p in zip(label, point):  #zip함수로 두 리스트 병렬로 반복 
            #l은 라벨 p는 좌표리스트 
            label_points[l].append(p) #l 라벨을 키로 가지는 아이템에 접근 from collections import defaultdict #키가 존재하지 않을 때 기본값을 생성하는 딕셔너리 

#키가 존재하지 않는 경우, defaultdict는 자동으로 빈 리스트를 생성, append(p): 현재 포인트
            
        cone_centers=[]
        for i in label_points: #딕셔너리에 저장된 각 라벨(i)에 대해 반복함 
            cone_points=label_points.get(i) #해당 라벨에 속한 모든 포인트의 리스트 반환
            x_list=[]  
            y_list=[]
            z_list=[]
            for k in cone_points: #각 포인트의 x,y,z 좌표를 리스트에 저장 
                x_list.append(k[0])
                y_list.append(k[1])
                z_list.append(k[2])
            x_range=max(x_list)-min(x_list)
            y_range=max(y_list)-min(y_list)
            z_range=max(z_list)-min(z_list)
            
            #if x_range>0.1 and x_range<0.55 and y_range>0.1 and y_range<0.55 and z_range>0.01 and z_range<0.95:
            if  z_range<1:
                x_mean=sum(x_list)/len(x_list)
                y_mean=sum(y_list)/len(y_list)
                cone_centers.append([x_mean,y_mean])
            
            # elif max(x_list)<3 and x_range>0.05 and x_range<0.55 and y_range>0.05 and y_range<0.55 and z_range<x_range/4 and z_range>0.05:
            #     x_mean=sum(x_list)/len(x_list)
            #     y_mean=sum(y_list)/len(y_list)
            #     cone_centers.append([x_mean,y_mean])     
                # print("ggggggooooooooooooooood")
                #이 조건은 왜 넣은거야??? 코드 짜신분께 여쭤보장            
            
        self.cones = PointCloud() #3d 점들의 집합을 나타내는 메시지 타입인 pointcloud의 객체를 생성함
        self.cones.header.frame_id='velodyne' #포인트 클라우드 데이터가 map 좌표계에 기반   

        for i in cone_centers: #콘 센터 리스트에 저장된 각 콘의 중심점에 대해 반복
            point=Point32()
            point.x=i[0]
            point.y=i[1]
            point.z=0.01 #원래 0임 
            self.cones.points.append(point) #위에 포인트를 selfconespoints 리스트에 추가

        self.cone_pub.publish(self.cones)
        print("콘의 개수: ",len(cone_centers))
        


def main():
    rospy.init_node('z_lidar_cone_detection',anonymous=True) 
    #anonymous 옵션으로 노드 이름에 무작위 숫자를 추가하여 고유하게 만듬
    Cone=Cone_detection()
    print("good")


    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    main()

"""
if __name__ == '__main__': 이 구문은 Python 스크립트가 "직접 실행될 때"만 코드 블록 내의 main() 함수를 실행하도록 하는 조건문입니다. Python에서 스크립트를 직접 실행하면, 스크립트의 __name__ 전역 변수는 '__main__'으로 설정됩니다. 하지만 해당 스크립트가 다른 스크립트에 의해 모듈로 임포트되어 사용될 때는, __name__은 원래의 모듈 이름(파일 이름)으로 설정됩니다.

이 방식을 사용함으로써, 스크립트가 다른 파일에 의해 임포트되었을 때 자동으로 실행되는 것을 방지하면서, 스크립트 파일이 직접 실행될 때 필요한 코드(여기서는 main() 함수)를 실행할 수 있습니다.

여기서 main() 함수는 프로그램의 시작점을 정의합니다. 즉, 스크립트가 직접 실행될 때 수행해야 할 모든 초기 설정, 객체 생성, 메인 루프 실행 등의 작업을 main() 함수 안에서 정의하고, 이 함수가 if __name__ == '__main__': 조건 아래에서 호출됩니다.
"""
