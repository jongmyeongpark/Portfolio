#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

class blue_cone:
    def __init__(self) -> None:
        '''
        Variable
        '''
        self.cone = []
    
    def get_cone_info(self, global_points: list):
        ''' it's sorted by sorting closed ERP-42 and global coordinate
        '''
        self.cone = global_points.copy()
        print('blue cone: ', self.cone)

    def calculate_each_cone_element(self):
        ''' subtract each cone vectors, adjacent element
        '''
        self.cone = np.asarray(self.cone)
        print('diff: ', np.diff(self.cone))

    def activate(self, global_points: list):
        print("*"*20)
        print("Activated Blue Class")
        
        self.get_cone_info(global_points)
        self.calculate_each_cone_element()
        
        print("*"*20)
        print()

class Fix:
    def __init__(self) -> None:
        '''
        Variable
            - self.current_pose_x, self.current_pose_y, self.current_pose_heading: current pose information
        Publisher

        Subscriber
            - self.point_assigned_sub: subscribe /point_assigned topics [local_x, local_y, label]

        '''
        # Variable
        self.current_pose_x = None
        self.currnet_pose_y = None
        self.current_pose_heading = None

        self.blue = blue_cone()

        # Publisher

        # Subscriber
        self.point_assigned_sub = rospy.Subscriber('/point_assigned', PointCloud2, self.point_assigned_callback, queue_size=1)
        self.current_pose = rospy.Subscriber('/current_pose', Point, self.current_pose_callback, queue_size=1)

    def point_assigned_callback(self, point: PointCloud2):
        ''' get point assigned topics [local_x, local_y, cone_label]
        '''        
        cone_list = []
        for p in pc2.read_points(point, field_names=("x", "y", "z"), skip_nans=True):
            cone_list.append([p[0], p[1], p[2]])  
        
        yellow_cone = [] 
        blue_cone = [] 

        sorted_cone_np = self.sorting_cone_list(cone_list)
        
        if len(cone_list) > 0:
            for local_x, local_y, label in sorted_cone_np:
                if label == 0:
                    blue_cone.append([local_x, local_y, label])
                
                elif label == 1:
                    yellow_cone.append([local_x, local_y, label])
        
        if len(blue_cone) > 0:
            global_blue_cone = self.transform_local2global(blue_cone)
            self.blue.activate(global_blue_cone)

        if len(yellow_cone) > 0:
            global_yellow_cone = self.transform_local2global(yellow_cone)

    def current_pose_callback(self, current_pose: Point):
        ''' get current pose [x, y, heading]
        '''
        self.current_pose_x = current_pose.x
        self.currnet_pose_y = current_pose.y
        self.current_pose_heading = current_pose.z

    def sorting_cone_list(self, cone_list: list) -> np.ndarray:
        ''' sorting cone list, using abs
        Arg:
            cone_list: raw point cloud
        
        Return:
            sorted_cone_list: sorted cone list

        '''
        abs_sum = []
        cone_np = np.asarray(cone_list)
        for local_x, local_y, _ in cone_list:
            local_x = abs(local_x)
            local_y = abs(local_y)
            abs_sum.append(local_x + local_y)

        abs_sum = np.asarray(abs_sum)
        index = np.argsort(abs_sum)
        sorted_cone_np = cone_np[index]

        return sorted_cone_np

    def transform_local2global(self, cone: list, gps_lidar_distance: float = 1.04):
        ''' local -> global, cone must be 3-dimensions [local_x, local_y, label]
        Arg:
            cone: [local_x, local_y, label] 
        
        Return:
            global_cone: [global_x, global_y, label]
        '''
        if self.current_pose_x is not None and self.currnet_pose_y is not None and self.current_pose_heading is not None:
            T = [[np.cos(self.current_pose_heading), -np.sin(self.current_pose_heading), self.current_pose_x],
                [np.sin(self.current_pose_heading), np.cos(self.current_pose_heading), self.currnet_pose_y],
                [0, 0, 1]]
            
            global_points = []
            for local_x, local_y, _ in cone:
                global_tm = np.dot(T, np.transpose([local_x+gps_lidar_distance, local_y, 1]))
                global_x = global_tm[0]
                global_y = global_tm[1]

                global_points.append([global_x, global_y])
            return global_points
        return []


def main():
    rospy.init_node('fix_cone', anonymous=True)
    fix = Fix()
    rospy.spin()

if __name__ == "__main__":
    main()
