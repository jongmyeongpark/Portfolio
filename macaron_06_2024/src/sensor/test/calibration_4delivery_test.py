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
from macaron_06.msg import lidar_info, Delivery, objects_info, object_info
from sklearn.cluster import DBSCAN  # 클러스터링 라이브러리 추가
'''
    delivery 전용 calibration코드 
'''
def get_x_rotation_matrix(radius: float) -> np.ndarray:
    x_sin = np.sin(radius)
    x_cos = np.cos(radius)
    m_x_rotate = np.eye(4, dtype=np.float32)
    m_x_rotate[:3, :3] = np.array([[1, 0, 0],
                                   [0, x_cos, -x_sin],
                                   [0, x_sin, x_cos]], dtype=np.float32)
    return m_x_rotate

def get_y_rotation_matrix(radius: float) -> np.ndarray:
    y_sin = np.sin(radius)
    y_cos = np.cos(radius)
    m_y_rotate = np.eye(4, dtype=np.float32)
    m_y_rotate[:3, :3] = np.array([[y_cos, 0, y_sin],
                                   [0, 1, 0],
                                   [-y_sin, 0, y_cos]], dtype=np.float32)
    return m_y_rotate

def get_z_rotation_matrix(radius: float) -> np.ndarray:
    z_sin = np.sin(radius)
    z_cos = np.cos(radius)
    m_z_rotate = np.eye(4, dtype=np.float32)
    m_z_rotate[:3, :3] = np.array([[z_cos, -z_sin, 0],
                                   [z_sin, z_cos, 0],
                                   [0, 0, 1]], dtype=np.float32)
    return m_z_rotate

def get_translate_matrix(x: float, y: float, z: float) -> np.ndarray:
    m_translate = np.eye(4, dtype=np.float32)
    m_translate[3, 0] = x
    m_translate[3, 1] = y
    m_translate[3, 2] = z

    return m_translate

def get_trans_matrix_lidar_to_camera3d(rotate, translate, rotate_again):
    rotate_x, rotate_y, rotate_z = np.deg2rad(rotate)
    rotate_again_x, rotate_again_y, rotate_again_z = np.deg2rad(rotate_again)
    translate_x, translate_y, translate_z = translate

    m_x_rotate = get_x_rotation_matrix(rotate_x)
    m_y_rotate = get_y_rotation_matrix(rotate_y)
    m_z_rotate = get_z_rotation_matrix(rotate_z)

    m_x_rotate_again = get_x_rotation_matrix(rotate_again_x)
    m_y_rotate_again = get_y_rotation_matrix(rotate_again_y)
    m_z_rotate_again = get_z_rotation_matrix(rotate_again_z)

    m_translate = get_translate_matrix(translate_x, translate_y, translate_z)

    return m_x_rotate @ m_y_rotate @ m_z_rotate @ m_translate @ m_y_rotate_again @ m_x_rotate_again
    # return m_x_rotate @ m_y_rotate @ m_z_rotate @ m_translate

def get_trans_matrix_camera3d_to_image_h1(img_shape) -> np.ndarray:
    aspect = img_shape[1] / img_shape[0]
    
    # 대각선 기준 
    fov = np.deg2rad(73.7)  #브리오 카메라 대각선 방향 fov값 65 78 90 3개중에 하나임  => 일단 90도로 #76
    # c290 웹캠 (신호등) : 대각선 fov78도 

    f = np.sqrt(1 + aspect ** 2) / np.tan(fov / 2)

    trans_matrix = np.array([[f, 0, 0, 0],
                             [0, f, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    return trans_matrix

# 가상의 이미지를 가로 2*aspect 높이 2라고 했으니깐 2를 나눠주는 과정 
def get_expand_matrix(img_height: float) -> np.ndarray:
    trans_matrix = np.eye(4)
    trans_matrix[0, 0] = img_height / 2
    trans_matrix[1, 1] = img_height / 2
    return trans_matrix

def project_lidar_to_screen(point_clouds: np.ndarray, img: np.ndarray, transform) -> np.ndarray:
    m_lidar_to_camera3d = get_trans_matrix_lidar_to_camera3d(transform[0], transform[1], transform[2]) 
    
    m_camera3d_to_camera2d = get_trans_matrix_camera3d_to_image_h1(img.shape)
    m_expand_image = get_expand_matrix(img.shape[0])
    trans_matrix = m_lidar_to_camera3d @ m_camera3d_to_camera2d @ m_expand_image

    point_clouds_without_intensity = np.hstack((point_clouds[:, :3], np.ones((point_clouds.shape[0], 1))))
    transposed_point_clouds = point_clouds_without_intensity @ trans_matrix

    transposed_point_clouds[:, :2] /= transposed_point_clouds[:, 2].reshape((-1, 1))

    img_height, img_width = img.shape[0], img.shape[1]
    transposed_point_clouds[:, 0] += img_width / 2
    transposed_point_clouds[:, 1] += img_height / 2

    index_of_fov = np.where((transposed_point_clouds[:, 0] < img_width) & (transposed_point_clouds[:, 0] >= 0) &
                            (transposed_point_clouds[:, 1] < img_height) & (transposed_point_clouds[:, 1] >= 0) &
                            (transposed_point_clouds[:, 2] > 0))[0]

    projected_point_clouds = transposed_point_clouds[index_of_fov, :]
    return projected_point_clouds, index_of_fov


class LidarCameraCalibration:
    def __init__(self):
        self.bridge = CvBridge()  
        self.lidar_sub = Subscriber('/cluster', lidar_info) # 임시
        # 실제로 multi_cma_4final.py 로 실행할 때는 이걸로 
        # self.camera_sub = Subscriber('/webcam_combined/image_raw', Image)
        # bag파일용 토픽은 combined_image
        self.camera_sub = Subscriber('combined_image', Image)
        self.cone_obj_sub = Subscriber('/delivery_obj_combined', Delivery)
        

        self.calibration_pub = rospy.Publisher('/calibration', Image, queue_size=1)  # 캘리브레이션 결과 이미지를 발행할 Publisher 객체를 생성
        
        self.points_assigned_pub = rospy.Publisher('/point_assigned', PointCloud2, queue_size=1)


        self.lidar_delivery_pub = rospy.Publisher('lidar_delivery_centroid_points', PointCloud2, queue_size=1)
        
        # LiDAR와 카메라 데이터를 동기화할 ApproximateTimeSynchronizer 객체를 생성
        self.sync = ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub, self.cone_obj_sub], queue_size=10, slop=100000000000000000000000,allow_headerless=True)
        self.sync.registerCallback(self.callback)  # 동기화된 데이터에 대해 콜백 함수 등록

        self.lidar_data = None  # LiDAR 데이터를 저장할 변수 초기화
        self.camera_image = None  # 카메라 이미지를 저장할 변수 초기화
        self.cone_info = []

        self.delivery_points = []

        self.label_str_to_int = {'delivery_a1': 0, 'delivery_a2': 1, 'delivery_a3': 2, 'delivery_b1': 3, 'delivery_b2': 4, 'delivery_b3': 5} # TODO: Enum 사용

    def callback(self, lidar_msg, img_msg, label_msg):
        # self.lidar_callback(lidar_msg)  # LiDAR 메시지를 처리하는 콜백 함수 호출
        self.delivery_cluster_callback(lidar_msg)
        self.camera_callback(img_msg)  # 카메라 메시지를 처리하는 콜백 함수 호출
        self.camera_delivery_cluster_callback(label_msg)
        self.process_calibration()  # 캘리브레이션을 수행하는 함수 호출
        print("callback")

    def camera_callback(self, data):
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # 카메라 메시지를 OpenCV 이미지로 변환하여 저장
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))  # 변환 중 오류 발생 시 에러 로그 출력

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

        # print(msg.delivery)

        if len(cluster_indices) == 0 or len(delivery_indices) == 0:
            rospy.loginfo("라이다 점 없데이 ")
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

        self.delivery_centroids = np.vstack(delivery_centroids)

        #라이다로 군집화한 배달표지판을 pub
        self.publish_lidar_delivery_point(self.delivery_centroids)

        # print("배달표지판 라이다 중심 점",len(self.delivery_centroids))

        # if self.delivery_centroids.shape[0] == 0:
        #     self.delivery_centroids = np.array([[0, 0, 0]])
        #     rospy.loginfo("라이다 점 없데이 ")
        #     self.publish_empty_point()

    #라이다로 군집화한 배달표지판을 pub
    def publish_lidar_delivery_point(self,points):
        # 필요한 헤더 및 포인트 필드 설정
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]


        cloud = point_cloud2.create_cloud(header, fields, points)

        self.lidar_delivery_pub.publish(cloud)
        # rospy.loginfo("lidar_delivery_centroid_points 토픽 퍼블리시 된데이")


    def publish_empty_point(self):
        # 필요한 헤더 및 포인트 필드 설정
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]

        # (-1, -1, -1) 점 생성 및 발행
        empty_point = [[-1, -1, -1]]
        empty_cloud = point_cloud2.create_cloud(header, fields, empty_point)
        
        self.points_assigned_pub.publish(empty_cloud)
        rospy.loginfo("[[-1, -1, -1]]   /point_assigned 토픽 퍼블리시 된데이")

    
    def camera_delivery_cluster_callback(self, label_msg):


        self.cone_info = []

            # label_msg.obj가 비어 있을 경우 처리
        if not label_msg.obj:
            self.publish_empty_point()
            return  # 더 이상 진행하지 않고 함수 종료
        
        for obj in label_msg.obj:
            label = obj.ns
            center_x = (obj.xmin + obj.xmax)/2
            center_y = (obj.ymin + obj.ymax)/2
            w = obj.xmax - obj.xmin
            h = obj.ymax - obj.ymin
            # print("x :",center_x,"y :" ,center_y, "  label: ",label)
        
            self.cone_info.append([label, center_x, center_y, w, h]) 

            

            start = (round(center_x - w/2), round(center_y - h/2))
            # end = (round(start[0] + w), round(start[1] + h))

            # 배달 용으로 바운딩 박스 실제 보다 h를 15늘려놓음 
            end = (round(start[0] + w), round(start[1] + h + 15))
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

            # 본선용 카메라 기준 
            # projected_points_left, projected_indices_left = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.11, 0.49, 0.25), (0, -27, 0)))
            # projected_points_right, projected_indices_right = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.11, 0.51, 0.25), (0, 27.5, 0)))
        
            # projected_points_left, projected_indices_left = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.22, 0.49, 0.25), (0, -27, 0)))
            # projected_points_right, projected_indices_right = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.22, 0.60, 0.20), (-1, 20, 0)))

            projected_points_left, projected_indices_left = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, :640, :], ((-90,90,0), (0.20, 0.54, 0.27), (3, -32, 0)))
            projected_points_right, projected_indices_right = project_lidar_to_screen(self.delivery_centroids, self.camera_image[:, 640:, :], ((-90,90,0), (-0.20, 0.54, 0.27), (-1, 20, 0)))

            points_left = self.delivery_centroids[projected_indices_left]
            points_right = self.delivery_centroids[projected_indices_right]
            projected_points_right[:, 0] += width / 2

            projected_points = np.vstack([projected_points_left, projected_points_right])
            for point in projected_points:
                x, y = int(point[0]), int(point[1])  
                cv2.circle(self.camera_image, (x, y), 10, (0, 0, 255), -1)  #중심점 빨간점

            try:
                # 캘리브레이션 결과 이미지를 ROS 이미지 메시지로 변환하여 발행
                self.calibration_pub.publish(self.bridge.cv2_to_imgmsg(self.camera_image, "bgr8"))
                # cv2.imshow("hello", self.camera_image)
                # cv2.waitKey(1)

            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))  # 변환 중 오류 발생 시 에러 로그 출력

        self.process_cone_calibration(np.vstack([points_left, points_right]), projected_points)
    
    def process_cone_calibration(self, points: np.ndarray, projected_points: np.ndarray):
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

            # for bbox in self.cone_info:
            #     label, center_x, center_y, w, h = bbox
            #     center = np.array([center_x, center_y])
            #     rect = ((center_x - w / 2, center_y - h / 2), (center_x + w / 2, center_y + h / 2))
                
            #     # print(rect[1][1]+10)

            #     point_indices_in_bbox = []
            #     for i, centroid in enumerate(projected_points):
            #         if (rect[0][0] < centroid[0] < rect[1][0]) and (rect[0][1] < centroid[1] < rect[1][1]+15):
            #             point_indices_in_bbox.append(i)


            for bbox in self.cone_info:
                label, center_x, center_y, w, h = bbox
                center = np.array([center_x, center_y])
                rect = ((center_x - w / 2, center_y - h / 2), (center_x + w / 2, center_y + h / 2))

                point_indices_in_bbox = []
                min_distance = float('inf')
                min_index = -1

                for i, centroid in enumerate(projected_points):
                    if (rect[0][0] < centroid[0] < rect[1][0]) and (rect[0][1] < centroid[1] < rect[1][1]+15):
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

                # # x 좌표가 중복되지 않도록 검사
                # if not any(p[0] == point[0] for p in points_assigned_bbox):
                #     points_assigned_bbox.append([point[0], point[1], self.label_str_to_int[label]])
                #     # print("표지판의 x,y,라벨 : ", points_assigned_bbox, "\n")

                #     pc2 = point_cloud2.create_cloud(header, fields, points_assigned_bbox)
                #     print("발행할 포인트:", points_assigned_bbox)
                #     self.points_assigned_pub.publish(pc2)


                # 여기서 점이 중복되지 않도록 검사
                if not any(p[:2] == point[:2] for p in points_assigned_bbox):
                    points_assigned_bbox.append([point[0], point[1], self.label_str_to_int[label]])
                
                # print("Tlqkf",points_assigned_bbox)

            if len(points_assigned_bbox) == 0:
                print("발행할 포인트가 없습니다.")
                self.publish_empty_point()
                
            else:
                print("발행할 포인트:", points_assigned_bbox)
                pc2 = point_cloud2.create_cloud(header, fields, points_assigned_bbox)
                self.points_assigned_pub.publish(pc2)

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