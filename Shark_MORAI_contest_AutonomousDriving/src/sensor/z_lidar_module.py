import numpy as np
import rospy
from numpy import array,transpose,dot,cross
from math import acos,sin,cos
import random
from sklearn.cluster import DBSCAN
from collections import defaultdict
from std_msgs.msg import String

class lidar_module:
    def __init__(self):
        #roi 설정
        self.bottom=-0.3 #음수, 라이다 위치부터 아래 몇미터까지인지
        self.above=2 #위 몇미터까지인지
        self.front=20 #몇미터 앞까지 볼건지
        self.width=5 #라이다로 볼 데이터의 폭 (2x라면 왼쪽으로x만큼 오른쪽으로 x만큼)
        self.behind=0 #양수, 라이다기준 몇미터 뒤까지 볼건지
        self.min_intensity=0 #세기
        
        self.roi=[self.bottom,self.above,self.front,self.width,self.min_intensity,self.behind]

        #dbscan 설정 https://saint-swithins-day.tistory.com/m/81, https://bcho.tistory.com/1205
        self.epsilon=0.2 #입실론 값 0.4
        self.min_points=4 #한 군집을 만들 최소 포인트 개수 4
        self.z_com_flag=True #z값 압축을 한다면 True를 사용해서 풀어줘야 함

        #voxel 설정 
        self.delta=0.01 #데이터가 delta의 배수로 나타나짐  

        #ransac 설정 https://gnaseel.tistory.com/33
        self.p_dist=0.1 #추출된 모델로부터 거리가 n이하이면 inlier로 취급함
        self.reps=100 #ransac 반복횟수
        self.p_angle=3.14/8 #라디안, 추출된 모델과 xy평면의 최대 각도 차이

    

    def intensity_voxel_roi(self,raw_data,roi=[-5,13,20,10,0,0],delta=0.01):
        """
        roi 설정
        bottom #음수, 라이다 위치부터 아래 몇미터까지인지
        above #위 몇미터까지인지
        front #몇미터 앞까지 볼건지
        width #라이다로 볼 데이터의 폭 (2x라면 왼쪽으로x만큼 오른쪽으로 x만큼)
        behind #양수, 라이다기준 몇미터 뒤까지 볼건지
        min_intensity #세기
        roi=[bottom,above,front,width,min_intensity,behind]

        voxel 설정
        delta voxel의 크기 m단위
        """
        self.roi=roi
        self.delta=delta
        def roi_map(raw_data):
            if -0.5*self.roi[3]<raw_data[1]<0.5*self.roi[3] and -1*self.roi[5]<raw_data[0]<self.roi[2] and self.roi[0]<raw_data[2]<self.roi[1]:
                x = (raw_data[0]//self.delta)*self.delta
                y = (raw_data[1]//self.delta)*self.delta
                z = (raw_data[2]//self.delta)*self.delta
                intensity = int(raw_data[3])
                return tuple([x,y,z,intensity])
            else:
                return tuple([0,0,0,0])   
        return list(map(roi_map,raw_data))  

    def new_voxel_roi(self,raw_data,roi=[-5,13,20,10,0,0],delta=0.01):
        """
        roi 설정
        bottom #음수, 라이다 위치부터 아래 몇미터까지인지
        above #위 몇미터까지인지
        front #몇미터 앞까지 볼건지
        width #라이다로 볼 데이터의 폭 (2x라면 왼쪽으로x만큼 오른쪽으로 x만큼)
        behind #양수, 라이다기준 몇미터 뒤까지 볼건지
        min_intensity #세기
        roi=[bottom,above,front,width,min_intensity,behind]

        voxel 설정
        delta voxel의 크기 m단위
        """
        self.roi=roi
        self.delta=delta
        def roi_map(raw_data):
            if -0.5*self.roi[3]<raw_data[1]<0.5*self.roi[3] and -1*self.roi[5]<raw_data[0]<self.roi[2] and self.roi[0]<raw_data[2]<self.roi[1]:
                x = (raw_data[0]//self.delta)*self.delta
                y = (raw_data[1]//self.delta)*self.delta
                z = (raw_data[2]//self.delta)*self.delta
                return tuple([x,y,z])
            else:
                return tuple([0,0,0])   
        return list(map(roi_map,raw_data))  
    
    def intensity_ransac(self,input_data,reps=100,p_dist=0.1,p_angle=3.14/8):
        """
        reps 반복회수
        p_dist 평면의 두께
        p_angle 평면의 최대 각도
        """
        self.reps=reps
        self.p_dist=p_dist
        self.p_angle=p_angle
        input_array=array(input_data)
        best_count=100000
        for j in range(self.reps):
            random_elements=random.sample(input_data,3)
            #뽑은 점 3개로 평면 만들기 위해 np array로 변경
            p1= array(random_elements[0])
            p2= array(random_elements[1])
            p3= array(random_elements[2])
            #평면을 이룰 벡터 2개 만들기
            v1=p2-p1
            v2=p3-p1
            #벡터 2개 크기 구하기
            v1_size=(v1[0]**2+v1[1]**2+v1[2]**2)**(1/2)            
            v2_size=(v2[0]**2+v2[1]**2+v2[2]**2)**(1/2)
            #벡터 2개 사잇각의 코사인 값
            cos_v1v2=dot(v1,v2)/(v1_size*v2_size)
            #이상하게 코사인인데 -1,1밖의 값이 가끔 나와서 변경
            if(cos_v1v2>1 or cos_v1v2<-1):
                cos_v1v2=0.99999
            #벡터 2개 사잇각
            angle_v1v2=acos(cos_v1v2)
            #평면의 단위법선벡터
            pppp=v1_size*v2_size*sin(angle_v1v2)
            if pppp == 0:
                pppp = 0.00001
            normal_vector=cross(v1,v2)/pppp
            #이것도 가끔 단위벡터의 z값인데 -1,1밖의 값이 나와서 변경
            if(normal_vector[2]>1 or normal_vector[2]<-1):
                normal_vector[2]=0.99999  
            
            #샘플링해서 구한 평면과 xy평면 사잇각의 크기      
            angle=acos(normal_vector[2])
            if angle<self.p_angle or angle>3.14-self.p_angle:
                ransac_count=0                
                #data 벡터화
                vector_array=input_array-p1
                #평면과의 거리 계산
                dist_array=abs(dot(vector_array,normal_vector))
                #평면 제거된 모델 추출
                weight_list=np.where(dist_array>self.p_dist,True,False)
                ransac_count=np.sum(weight_list)

                #센 점 개수로 가장 좋은 모델 추출
                if ransac_count < best_count:
                    best_count=ransac_count
                    best_weight_list=weight_list 

        best_no_land_model=input_array[best_weight_list]
        return best_no_land_model, best_weight_list

    def ransac(self,input_data,reps=100,p_dist=0.1,p_angle=3.14/8):
        """
        reps 반복회수
        p_dist 평면의 두께
        p_angle 평면의 최대 각도
        """
        self.reps=reps
        self.p_dist=p_dist
        self.p_angle=p_angle
        input_array=array(input_data)
        best_count=100000
        try:
            for j in range(self.reps):
                random_elements=random.sample(input_data,3)
                #뽑은 점 3개로 평면 만들기 위해 np array로 변경
                p1= array(random_elements[0])
                p2= array(random_elements[1])
                p3= array(random_elements[2])
                #평면을 이룰 벡터 2개 만들기
                v1=p2-p1
                v2=p3-p1
                #벡터 2개 크기 구하기
                v1_size=(v1[0]**2+v1[1]**2+v1[2]**2)**(1/2)            
                v2_size=(v2[0]**2+v2[1]**2+v2[2]**2)**(1/2)
                #벡터 2개 사잇각의 코사인 값
                cos_v1v2=dot(v1,v2)/(v1_size*v2_size)
                #이상하게 코사인인데 -1,1밖의 값이 가끔 나와서 변경
                if(cos_v1v2>1 or cos_v1v2<-1):
                    cos_v1v2=0.99999
                #벡터 2개 사잇각
                angle_v1v2=acos(cos_v1v2)
                #평면의 단위법선벡터
                pppp=v1_size*v2_size*sin(angle_v1v2)
                if pppp == 0:
                    pppp = 0.00001
                normal_vector=cross(v1,v2)/pppp
                #이것도 가끔 단위벡터의 z값인데 -1,1밖의 값이 나와서 변경
                if(normal_vector[2]>1 or normal_vector[2]<-1):
                    normal_vector[2]=0.99999  
                
                #샘플링해서 구한 평면과 xy평면 사잇각의 크기      
                angle=acos(normal_vector[2])
                if angle<self.p_angle or angle>3.14-self.p_angle:
                    ransac_count=0                
                    #data 벡터화
                    vector_array=input_array-p1
                    #평면과의 거리 계산
                    dist_array=abs(dot(vector_array,normal_vector))
                    #평면 제거된 모델 추출
                    weight_list=np.where(dist_array>self.p_dist,True,False)
                    ransac_count=np.sum(weight_list)

                    #센 점 개수로 가장 좋은 모델 추출
                    if ransac_count < best_count:
                        best_count=ransac_count
                        best_weight_list=weight_list 

            best_no_land_model=input_array[best_weight_list]
            return best_no_land_model
        except:
           return array(input_data)


    def z_compressor(self,input_data):
        def z_com(input_point):
            input_point[2]=input_point[2]*self.epsilon*10000/(self.front*456)
            return input_point
        input_data=list(map(z_com,input_data))
        return input_data
    

    def z_compressor_open3d(self, input_data):
        def z_com(input_point):
            input_point[2] = input_point[2] * self.epsilon * 10000 / (self.front * 456)
            return input_point
        # 이미 input_data는 리스트 형태의 점들이므로, 직접 z_com 함수를 적용합니다.
        compressed_data = list(map(z_com, input_data))
        return compressed_data


    def dbscan(self,input_data,epsilon=0.1,min_points=4):   #입실론 0.2인데 바꿈 
        self.epsilon=epsilon
        self.min_points=min_points
        #eps과 min_points가 입력된 모델 생성
        model=DBSCAN(eps=self.epsilon, min_samples=self.min_points, n_jobs=-11) # grid search나 베이지안으로 최적화 인자값 찾을 수 있음
        #데이터를 라이브러리가 읽을 수 있게 np array로 변환
        DB_Data=np.array(input_data,dtype=object)
        if len(DB_Data)<1:
            DB_Data=np.array([[0,0,0]])
        #모델 예측
        labels=model.fit_predict(DB_Data)
        
        k=0
        
        no_noise_model=[]
        no_noise_label=[]
        for i in input_data:
            if labels[k] != -1 :
                if self.z_com_flag is True:
                    z=i[2]*(self.front*456)/(self.epsilon*10000)
                no_noise_model.append([i[0],i[1],z])
                no_noise_label.append(labels[k])
            k+=1
        return no_noise_model, no_noise_label

     
    def intensity_dbscan(self,input_data,intensity_list,epsilon,min_points): #epsilon=0.2,min_points=4 
        self.epsilon=epsilon
        self.min_points=min_points
        #eps과 min_points가 입력된 모델 생성
        model=DBSCAN(eps=self.epsilon, min_samples=self.min_points)
        #데이터를 라이브러리가 읽을 수 있게 np array로 변환
        DB_Data=np.array(input_data, dtype=object)
        #모델 예측
        labels=model.fit_predict(DB_Data)
        k=0
        no_noise_model=[]
        no_noise_label=[]
        no_noise_intensity=[]
        for i in input_data:
            if labels[k] != -1 : #노이즈가 아니라면 
                z=i[2]*(self.front*456)/(self.epsilon*10000) #z 압축했던거 다시 풀어주는 과정 
                no_noise_model.append([i[0],i[1],z]) #x,y,z 저장
                no_noise_label.append(labels[k]) #해당 point의 labels 저장 
                no_noise_intensity.append(intensity_list[k]) #i[3]으로 써도 됩니다
            k+=1
        return no_noise_model, no_noise_label, no_noise_intensity
    
    '''
    DBSCAN으로부터 반환된 labels 배열에서, 라벨이 -1이 아닌 경우(즉, 노이즈가 아닌 경우)만 필터링하여 no_noise_model, no_noise_label, no_noise_intensity 리스트에 추가합니다.
    이 과정에서도 x,y,z 좌표와 해당 강도 값의 인덱스 대응 관계는 변하지 않습니다. k 인덱스를 사용하여 input_data와 intensity_list에서 동일한 위치의 데이터를 참조하기 때문입니다.
    '''

    def cone_detector(self,labels, points3D):
        #cone_detect
        label_points = defaultdict(list)
        for l, p in zip(points3D, labels):
            label_points[l].append(p)

        cone_centers=[]
        for i in label_points:
            cone_points=label_points.get(i)
            x_list=[]
            y_list=[]
            z_list=[]
            for k in cone_points:
                x_list.append(k[0])
                y_list.append(k[1])
                z_list.append(k[2])
            x_range=max(x_list)-min(x_list)
            y_range=max(y_list)-min(y_list)
            z_range=max(z_list)-min(z_list)
            
            if x_range>=0 and x_range<0.55 and y_range>=0 and y_range<0.55 and z_range>=0 and z_range<1:
                x_mean=sum(x_list)/len(x_list)
                y_mean=sum(y_list)/len(y_list)
                z_mean=sum(z_list)/len(z_list)
                cone_centers.append([x_mean,y_mean,z_mean])
            elif max(x_list)<3 and x_range>0.05 and x_range<0.55 and y_range>0.05 and y_range<0.55 and z_range<x_range/4 and z_range>0.05:
                x_mean=sum(x_list)/len(x_list)
                y_mean=sum(y_list)/len(y_list)
                z_mean=sum(z_list)/len(z_list)
                cone_centers.append([x_mean,y_mean,z_mean])
                
        return cone_centers  


    def tf2tm(self,no_z_points,x,y,heading):
        obs_tm=np.empty((1,3))
        rgb_values = np.empty((1,1))  # New array for rgb values

        T = [[cos(heading), -1*sin(heading), x], \
                [sin(heading),  cos(heading), y], \
                [      0     ,      0       , 1]] 
        
        for point in no_z_points:
            obs_tm = np.append(obs_tm,[dot(T,transpose([point[0]+1.11, point[1],1]))],axis=0) # point[0] -> 객체를 subscribe할 수 없음 오류
            
            # rgb_value = point[3]  # Replace with actual rgb value extraction
            # rgb_values = np.append(rgb_values, [[rgb_value]], axis=0)

        obs_tm[:,2]=0
        obs_tm=obs_tm[1:]

        # rgb_values = rgb_values[1:]  # Remove the first row (which was empty)

        # obs_tm = np.hstack((obs_tm, rgb_values))  # Add the rgb_v
        
        return obs_tm