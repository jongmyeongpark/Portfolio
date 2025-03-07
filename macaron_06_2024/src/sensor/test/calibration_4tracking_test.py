#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, CompressedImage, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs import point_cloud2
from macaron_06.msg import lidar_info, Cone, objects_info, object_info
from sklearn.cluster import DBSCAN  # 클러스터링 라이브러리 추가
from calibration_module import calibration_module


class LidarCameraCalibration:
    def __init__(self):
        self.bridge = CvBridge()  

        self.lidar_sub = Subscriber('/cluster', lidar_info) # 임시
        self.cone_obj_sub = Subscriber('cone_obj_combined', Cone)

        self.calibration_pub = rospy.Publisher('/calibration', Image, queue_size=1)  # 캘리브레이션 결과 이미지를 발행할 Publisher 객체를 생성
        self.points_assigned_pub = rospy.Publisher('/point_assigned', PointCloud2, queue_size=1)
        self.lidar_pub = rospy.Publisher('/cluster_cone', lidar_info, queue_size=1)


        self.sync = ApproximateTimeSynchronizer([self.lidar_sub, self.cone_obj_sub], queue_size=10, slop=0.1,allow_headerless=True)
        self.sync.registerCallback(self.callback)  # 동기화된 데이터에 대해 콜백 함수 등록

        self.lidar_data = None  # LiDAR 데이터를 저장할 변수 초기화
        self.cone_info = []

        self.label_str_to_int = {'blue_cone': 0, 'yellow_cone': 1} # TODO: Enum 사용

        self.calib = calibration_module()

        self.camera_image = np.zeros((480, 1280, 3), dtype=np.uint8) #초기화 


    def callback(self, lidar_msg, label_msg):
        print("callback")
        self.cone_callback(lidar_msg)
        self.cone_obj_callback(label_msg)
        self.process_calibration()  # 캘리브레이션을 수행하는 함수 호출


    def cone_callback(self, msg):
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return
        
        self.stamp = msg.header.stamp

        cluster_indices = list(msg.clusters)
        cone_indices = list(msg.cones)

        if len(cluster_indices) == 0 or len(cone_indices) == 0:
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        cones = [clusters[i] for i in cone_indices]
        self.lidar_data = np.vstack(cones)
        

        #cone 중심점 
        cone_centroids = []
        for cone in cones:
            cone_centroids.append(np.mean(cone, axis=0))

        # 3차원 점 
        self.cone_centroids = np.vstack(cone_centroids)

        # 이미지 (480, 1280, 3) 전부다 0으로 
        self.camera_image[:] = (0, 0, 0)

        self.lidar_pub.publish(msg) 

    
    def cone_obj_callback(self, label_msg):
        self.cone_info = []
        for obj in label_msg.obj:
            label = obj.ns
            center_x = (obj.xmin + obj.xmax)/2
            center_y = (obj.ymin + obj.ymax)/2
            w = obj.xmax - obj.xmin
            h = obj.ymax - obj.ymin
        
            self.cone_info.append([label, center_x, center_y, w, h]) 
            start = (round(center_x - w/2), round(center_y - h/2))
            end = (round(start[0] + w), round(start[1] + h))

            if self.camera_image is not None: 
                cv2.rectangle(self.camera_image, start, end, (0, 255, 0), 4)
   
   
    def process_calibration(self):
        if self.lidar_data is not None and self.camera_image is not None:  # LiDAR 데이터와 카메라 이미지가 모두 존재하는 경우
            # LiDAR 포인트를 카메라 이미지에 투영하여 반환
            height, width, channel = self.camera_image.shape

            # projected_points_left : [[581.07831691 121.02133798   9.95936957   1.        ]]
            # projected_indices_left : [3 4 5 7 8]
            projected_points_left, projected_indices_left = self.calib.project_lidar_to_screen(self.cone_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.22, 0.54, 0.27), (-19.5, -44.9, 1)))
            projected_points_right, projected_indices_right = self.calib.project_lidar_to_screen(self.cone_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.22, 0.54, 0.27), (-20, 33.6, 1)))
 
            projected_points_right[:, 0] += width / 2

            #pionts_left : [ [10.36137962  3.87201854 -0.77304044]]
            points_left = self.cone_centroids[projected_indices_left]
            points_right = self.cone_centroids[projected_indices_right]


            points_3d_cone = np.vstack([points_left, points_right])

            projected_points = np.vstack([projected_points_left, projected_points_right])

            for point in projected_points:
                x, y = int(point[0]), int(point[1])  # 투영된 포인트의 좌표를 정수로 변환
                cv2.circle(self.camera_image, (x, y), 7, (0, 0, 255), -1)  # 이미지에 투영된 포인트를 빨간색 원으로 그림

                # 캘리브레이션 결과 이미지를 ROS 이미지 메시지로 변환하여 발행
                self.calibration_pub.publish(self.bridge.cv2_to_imgmsg(self.camera_image, "bgr8"))

        self.process_cone_calibration(points_3d_cone, projected_points)
    

    def process_cone_calibration(self, points_3d_cone: np.ndarray, projected_points: np.ndarray):
            if len(projected_points) == 0:
                return
            

            points_assigned_bbox = []
            header = Header()
            header.stamp = self.stamp
            header.frame_id = "velodyne"
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]
            
            projected_points = projected_points[:, :2]
            
            # 원래 버전 
            # for bbox in self.cone_info:
            #     label, center_x, center_y, w, h = bbox
            #     center = np.array([center_x, center_y])
            #     rect = ((center_x - w / 2, center_y - h / 2), (center_x + w / 2, center_y + h / 2))

            #     point_indices_in_bbox = []
            #     for i, centroid in enumerate(projected_points):
            #         if (rect[0][0] < centroid[0] < rect[1][0]) and (rect[0][1] < centroid[1] < rect[1][1]):
            #             point_indices_in_bbox.append(i)

            # 바운딩 박스 안에 2개의 점이 찍히는 경우 예외처리   
            for bbox in self.cone_info:
                label, center_x, center_y, w, h = bbox
                center = np.array([center_x, center_y])
                rect = ((center_x - w / 2, center_y - h / 2), (center_x + w / 2, center_y + h / 2))

                point_indices_in_bbox = []
                min_distance = float('inf')
                min_index = -1

                for i, centroid in enumerate(projected_points):
                    if (rect[0][0] < centroid[0] < rect[1][0]) and (rect[0][1] < centroid[1] < rect[1][1]):
                        # 계산된 거리 (x^2 + y^2)
                        distance = points_3d_cone[i][0]**2 + points_3d_cone[i][1]**2
                        # 최소 거리 업데이트
                        if distance < min_distance:
                            min_distance = distance
                            min_index = i

                # 최소 거리를 가진 점의 인덱스만 추가
                if min_index != -1:
                    point_indices_in_bbox.append(min_index)          


                if len(point_indices_in_bbox) == 0:
                    continue

                diff = np.linalg.norm(projected_points[point_indices_in_bbox] - center, axis=1)
                min_index = np.argmin(diff)

                point = points_3d_cone[point_indices_in_bbox[min_index]].tolist()

                # x 좌표가 중복되지 않도록 검사
                if not any(p[0] == point[0] for p in points_assigned_bbox):
                    points_assigned_bbox.append([point[0], point[1], self.label_str_to_int[label]])
                    # print("표지판의 x,y,라벨 : ", points_assigned_bbox, "\n")

                    pc2 = point_cloud2.create_cloud(header, fields, points_assigned_bbox)
                    self.points_assigned_pub.publish(pc2)

                    
 
if __name__ == '__main__':
    rospy.init_node("calibaration")
    try:
        calibration = LidarCameraCalibration()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            calibration
            rate.sleep()
    except rospy.ROSInterruptException:
        pass