#!/usr/bin/env python

import numpy as np 


class calibration_module:
    def __init__(self):
        
        pass

    def get_x_rotation_matrix(self, radius: float) -> np.ndarray:
        x_sin = np.sin(radius)
        x_cos = np.cos(radius)
        m_x_rotate = np.eye(4, dtype=np.float32)
        m_x_rotate[:3, :3] = np.array([[1, 0, 0],
                                    [0, x_cos, -x_sin],
                                    [0, x_sin, x_cos]], dtype=np.float32)
        return m_x_rotate

    def get_y_rotation_matrix(self, radius: float) -> np.ndarray:
        y_sin = np.sin(radius)
        y_cos = np.cos(radius)
        m_y_rotate = np.eye(4, dtype=np.float32)
        m_y_rotate[:3, :3] = np.array([[y_cos, 0, y_sin],
                                    [0, 1, 0],
                                    [-y_sin, 0, y_cos]], dtype=np.float32)
        return m_y_rotate

    def get_z_rotation_matrix(self, radius: float) -> np.ndarray:
        z_sin = np.sin(radius)
        z_cos = np.cos(radius)
        m_z_rotate = np.eye(4, dtype=np.float32)
        m_z_rotate[:3, :3] = np.array([[z_cos, -z_sin, 0],
                                    [z_sin, z_cos, 0],
                                    [0, 0, 1]], dtype=np.float32)
        return m_z_rotate

    def get_translate_matrix(self, x: float, y: float, z: float) -> np.ndarray:
        m_translate = np.eye(4, dtype=np.float32)
        m_translate[3, 0] = x
        m_translate[3, 1] = y
        m_translate[3, 2] = z

        return m_translate

    def get_trans_matrix_lidar_to_camera3d(self, rotate, translate, rotate_again):
        rotate_x, rotate_y, rotate_z = np.deg2rad(rotate)
        rotate_again_x, rotate_again_y, rotate_again_z = np.deg2rad(rotate_again)
        translate_x, translate_y, translate_z = translate

        m_x_rotate = self.get_x_rotation_matrix(rotate_x)
        m_y_rotate = self.get_y_rotation_matrix(rotate_y)
        m_z_rotate = self.get_z_rotation_matrix(rotate_z)

        m_x_rotate_again = self.get_x_rotation_matrix(rotate_again_x)
        m_y_rotate_again = self.get_y_rotation_matrix(rotate_again_y)
        m_z_rotate_again = self.get_z_rotation_matrix(rotate_again_z)

        m_translate = self.get_translate_matrix(translate_x, translate_y, translate_z)

        return m_x_rotate @ m_y_rotate @ m_z_rotate @ m_translate @ m_y_rotate_again @ m_x_rotate_again @ m_z_rotate_again
        # return m_x_rotate @ m_y_rotate @ m_z_rotate @ m_translate

    def get_trans_matrix_camera3d_to_image_h1(self, img_shape) -> np.ndarray:
        aspect = img_shape[1] / img_shape[0]
        
        # 대각선 기준 
        fov = np.deg2rad(73.7) 

        f = np.sqrt(1 + aspect ** 2) / np.tan(fov / 2)

        trans_matrix = np.array([[f, 0, 0, 0],
                                [0, f, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        return trans_matrix

    # 가상의 이미지를 가로 2*aspect 높이 2라고 했으니깐 2를 나눠주는 과정 
    def get_expand_matrix(self, img_height: float) -> np.ndarray:
        trans_matrix = np.eye(4)
        trans_matrix[0, 0] = img_height / 2
        trans_matrix[1, 1] = img_height / 2
        return trans_matrix

    def project_lidar_to_screen(self, point_clouds: np.ndarray, img: np.ndarray, transform) -> np.ndarray:
        m_lidar_to_camera3d = self.get_trans_matrix_lidar_to_camera3d(transform[0], transform[1], transform[2]) 
        
        m_camera3d_to_camera2d = self.get_trans_matrix_camera3d_to_image_h1(img.shape)
        m_expand_image = self.get_expand_matrix(img.shape[0])
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