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
from macaron_06.msg import lidar_info, Delivery
from sklearn.cluster import DBSCAN  # 클러스터링 라이브러리 추가
from final.calibration_module import calibration_module
# from calibration_module import calibration_module

class LidarCameraCalibration:
    def __init__(self):
        self.bridge = CvBridge()  
        self.lidar_sub = Subscriber('/cluster', lidar_info) 
        self.cone_obj_sub = Subscriber('/delivery_obj_combined', Delivery)
        
        # self.calibration_pub = rospy.Publisher('/calibration', Image, queue_size=1) 
        # self.points_assigned_pub = rospy.Publisher('/point_assigned', PointCloud2, queue_size=1)
        
        # LiDAR와 카메라 데이터를 동기화할 ApproximateTimeSynchronizer 객체를 생성
        self.sync = ApproximateTimeSynchronizer([self.lidar_sub, self.cone_obj_sub], queue_size=10, slop=0.1,allow_headerless=True)
        self.sync.registerCallback(self.callback)  # 동기화된 데이터에 대해 콜백 함수 등록

        self.lidar_data = None  # LiDAR 데이터를 저장할 변수 초기화
        self.camera_image = None  # 카메라 이미지를 저장할 변수 초기화
        self.delivery_info = []
        self.delivery_points = []
        self.label_str_to_int = {'delivery_a1': 0, 'delivery_a2': 1, 'delivery_a3': 2, 'delivery_b1': 3, 'delivery_b2': 4, 'delivery_b3': 5} # TODO: Enum 사용
        self.camera_image = np.zeros((480, 1280, 3), dtype=np.uint8) #초기화 
        self.calib = calibration_module()


    def callback(self, lidar_msg, label_msg):
        self.delivery_cluster_callback(lidar_msg)
        self.camera_delivery_cluster_callback(label_msg)
        self.process_calibration()  # 캘리브레이션을 수행하는 함수 호출

        # print("callback")
        self.calib_point() #calibration 점 보내줍니다
        

    def calib_point(self):
        point = []
        point = self.publish_empty_point()
        point = self.process_calibration()

        if not point:
            point = [[-1,-1,-1]]
        
        point_array = np.array(point)
        # print(point_array)

        return point


    def delivery_cluster_callback(self, msg):
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return
        
        self.stamp = msg.header.stamp

        cluster_indices = list(msg.clusters)
        delivery_indices = list(msg.delivery) #임시방편 라이다 코드 바꾸면 cones -> delivery 

        if len(cluster_indices) == 0 or len(delivery_indices) == 0:
            # rospy.loginfo("라이다 점 없데이 ")
            # self.delivery_centroids = np.array([[-1, -1, -1]])
            self.publish_empty_point()
            return

        clusters = []
        count = 0
        
        for indice_size in msg.clusterSize:
            indice = cluster_indices[count : count+indice_size]
            count += indice_size

            clusters.append(pcd[indice, :])

        deliveries = [clusters[i] for i in delivery_indices]
        self.lidar_data = np.vstack(deliveries)
        
        #표지판(delivery) 중심점 
        delivery_centroids = []
        for delivery in deliveries:
            delivery_centroids.append(np.mean(delivery, axis=0))

        self.delivery_centroids = []
        self.delivery_centroids = np.vstack(delivery_centroids)
        
        # 이미지 (480, 1280, 3) 전부다 0으로 
        self.camera_image[:] = (0, 0, 0)


    def publish_empty_point(self):
        empty_point = [[-1, -1, -1]]

        return empty_point

    
    def camera_delivery_cluster_callback(self, label_msg):
        self.delivery_info = []
        # label_msg.obj가 비어 있을 경우 처리
        if not label_msg.obj:
            self.publish_empty_point()
            return 
        
        for obj in label_msg.obj:
            label = obj.ns
            center_x = (obj.xmin + obj.xmax)/2
            center_y = (obj.ymin + obj.ymax)/2
            w = obj.xmax - obj.xmin
            h = obj.ymax - obj.ymin
            self.delivery_info.append([label, center_x, center_y, w, h]) 
            start = (round(center_x - w/2), round(center_y - h/2))
            
            # 배달 용으로 바운딩 박스 실제 보다 h를 40늘려놓음 
            end = (round(start[0] + w), round(start[1] + h + 40))
            if self.camera_image is not None: 
                cv2.rectangle(self.camera_image, start, end, (0, 255, 0), 4)
   

    def process_calibration(self):
        points_left = []
        points_right = []
        projected_points = []


        if self.lidar_data is not None and self.camera_image is not None:  # LiDAR 데이터와 카메라 이미지가 모두 존재하는 경우
            # LiDAR 포인트를 카메라 이미지에 투영하여 반환
            height, width, channel = self.camera_image.shape
            # print(self.camera_image.shape)

            projected_points_left, projected_indices_left = self.calib.project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.20, 0.52, 0.3), (1, -19.2, 0)))
            projected_points_right, projected_indices_right = self.calib.project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.20, 0.52, 0.3), (-1, 17.5, 0)))

            # # # 배달표지판_cone_0825.bag 기준 
            # projected_points_left, projected_indices_left = self.calib.project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.20, 0.52, 0.3), (0, -25, -2)))
            # projected_points_right, projected_indices_right = self.calib.project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.20, 0.52, 0.3), (2, 24, 0)))

            # points_left = self.delivery_centroids[projected_indices_left]
            # points_right = self.delivery_centroids[projected_indices_right]


            # 유효한 인덱스만 사용하여 points_left와 points_right를 설정합니다.
            if projected_indices_left.size > 0 and np.max(projected_indices_left) < len(self.delivery_centroids):
                points_left = self.delivery_centroids[projected_indices_left]

            if projected_indices_right.size > 0 and np.max(projected_indices_right) < len(self.delivery_centroids):
                points_right = self.delivery_centroids[projected_indices_right]

            projected_points_right[:, 0] += width / 2

            projected_points = np.vstack([projected_points_left, projected_points_right])
            for point in projected_points:
                x, y = int(point[0]), int(point[1])  
                cv2.circle(self.camera_image, (x, y), 10, (0, 0, 255), -1)  #중심점 빨간점

            # 이미지 보고싶으면 주석 푸세요
            # try:
            #     # 캘리브레이션 결과 이미지를 ROS 이미지 메시지로 변환하여 발행
            #     self.calibration_pub.publish(self.bridge.cv2_to_imgmsg(self.camera_image, "bgr8"))
            #     # cv2.imshow("hello", self.camera_image)
            #     # cv2.waitKey(1)

            # except CvBridgeError as e:
            #     rospy.logerr("CvBridge Error: {0}".format(e))  

            combined = np.array([])
            if np.array(points_left).size == 0:
                combined = points_right
            elif np.array(points_right).size == 0:
                combined = points_left
            else:
                combined = np.vstack((points_left, points_right))
            # self.process_cone_calibration(np.vstack([points_left, points_right]), projected_points)
            self.process_cone_calibration(combined, projected_points)



            assigned_point = self.process_cone_calibration(combined, projected_points)
            
            return assigned_point
        


    def process_cone_calibration(self, points: np.ndarray, projected_points: np.ndarray):
            if len(projected_points) == 0:
                return

            points_assigned_bbox = []
            # header = Header()
            # header.stamp = self.stamp
            # header.frame_id = "velodyne"
            # fields = [PointField('x', 0, PointField.FLOAT32, 1),
            #     PointField('y', 4, PointField.FLOAT32, 1),
            #     PointField('z', 8, PointField.FLOAT32, 1),
            #     ]
            
            projected_points = projected_points[:, :2]

            for bbox in self.delivery_info:
                label, center_x, center_y, w, h = bbox
                center = np.array([center_x, center_y])
                rect = ((center_x - w / 2, center_y - h / 2), (center_x + w / 2, center_y + h / 2))

                point_indices_in_bbox = []
                min_distance = float('inf')
                min_index = -1

                for i, centroid in enumerate(projected_points):
                    if (rect[0][0] < centroid[0] < rect[1][0]) and (rect[0][1] < centroid[1] < rect[1][1]+40):
                        # 계산된 거리 (x^2 + y^2)
                        distance = points[i][0]**2 + points[i][1]**2
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

                point = points[point_indices_in_bbox[min_index]].tolist()

                # point_ratio 추가 : 가장 왼쪽이 0 가장 오른쪽이 1
                projected_point = projected_points[point_indices_in_bbox[min_index]].tolist()
                point_ratio = ((projected_point[0] - center_x) / w)+ 0.5
                
                # 여기서 점이 중복되지 않도록 검사
                if not any(p[:2] == point[:2] for p in points_assigned_bbox):
                    points_assigned_bbox.append([point[0], point[1], self.label_str_to_int[label],point_ratio])

            if len(points_assigned_bbox) == 0:
                self.publish_empty_point()
                
            else:
                
                # pc2 = point_cloud2.create_cloud(header, fields, points_assigned_bbox)
                # self.points_assigned_pub.publish(pc2)

                return points_assigned_bbox
    

if __name__ == '__main__':
    rospy.init_node("calibaration_4deliery")
    try:
        calibration = LidarCameraCalibration()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            calibration
            rate.sleep()
    except rospy.ROSInterruptException:
        pass