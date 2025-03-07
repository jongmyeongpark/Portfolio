#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from message_filters import Subscriber
from sensor_msgs import point_cloud2
from macaron_06.msg import lidar_info
import time

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

    return m_x_rotate @ m_y_rotate @ m_z_rotate @ m_translate @ m_y_rotate_again @ m_x_rotate_again @m_z_rotate_again

def get_trans_matrix_camera3d_to_image_h1(img_shape) -> np.ndarray:
    aspect = img_shape[1] / img_shape[0]
    
    fov = np.deg2rad(73.7)

    f = np.sqrt(1 + aspect ** 2) / np.tan(fov / 2)

    trans_matrix = np.array([[f, 0, 0, 0],
                             [0, f, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    return trans_matrix

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
        self.bridge = CvBridge()  # CvBridge 객체를 생성하여 ROS 이미지 메시지와 OpenCV 이미지 간의 변환을 처리
        self.lidar_sub = Subscriber('/cluster', lidar_info) # LiDAR 데이터를 구독할 Subscriber 객체를 생성 (임시)
        self.camera_sub = Subscriber('/webcam_combined/image_raw', Image)  # 카메라 데이터를 구독할 Subscriber 객체를 생성
        self.calibration_pub = rospy.Publisher('/calibration', Image, queue_size=1)  # 캘리브레이션 결과 이미지를 발행할 Publisher 객체를 생성
        
        self.lidar_times = []
        self.camera_times = []
        self.lidar_data = None
        self.camera_image = None
        self.slop = 0.1

        self.lidar_sub.registerCallback(self.lidar_callback)
        self.camera_sub.registerCallback(self.camera_callback)

    def lidar_callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        self.lidar_times.append(current_time)
        if len(self.lidar_times) > 1:
            interval = self.lidar_times[-1] - self.lidar_times[-2]
            if interval > self.slop:
                self.slop = interval
        self.cone_callback(msg)

    def camera_callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        self.camera_times.append(current_time)
        if len(self.camera_times) > 1:
            interval = self.camera_times[-1] - self.camera_times[-2]
            if interval > self.slop:
                self.slop = interval
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_calibration()

    def cone_callback(self, msg):
        pcd = []
        for point in point_cloud2.read_points(msg.data, field_names=("x", "y", "z", "intensity")):
            pcd.append(point)
        pcd = np.array(pcd)[:, :3]
        
        if pcd.shape[0] == 0:
            return

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
        cones = np.vstack(cones)
        self.lidar_data = cones

    def process_calibration(self):
        if self.lidar_data is not None and self.camera_image is not None:
            height, width, channel = self.camera_image.shape

            projected_points_left, _ = project_lidar_to_screen(self.lidar_data, self.camera_image[:, :640, :], ((-90,90,0), (0.12, 0.52, 0.30), (1, -19.2, 0)))
            projected_points_right, _ = project_lidar_to_screen(self.lidar_data, self.camera_image[:, 640:, :], ((-90,90,0), (-0.12, 0.52, 0.30), (-1, 17.5, 0)))                        
            projected_points_right[:, 0] += width / 2

            projected_points = np.vstack([projected_points_left, projected_points_right])
            
            for point in projected_points:
                x, y = int(point[0]), int(point[1])
                cv2.circle(self.camera_image, (x, y), 2, (0, 255, 0), -1)

            try:
                self.calibration_pub.publish(self.bridge.cv2_to_imgmsg(self.camera_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('lidar_camera_calibration', anonymous=True)
    calibration = LidarCameraCalibration()
    rospy.spin()
