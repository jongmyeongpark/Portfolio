# Camera-LiDAR Sensor fusion
Sensor fusion for rubber cone
<p align="center">
  <img src="https://github.com/user-attachments/assets/e50b95db-2dc3-42c3-b49c-0175d75fc592" alt="image">
</p>


## Introduction
Calibration is performed using the topics in the images on the left and right and the topics you receive from the lidar.

However, sending an image topic is very expensive to calculate, so you should be careful.

CMakeLists.txt is not uploaded separately because users can fill it out as needed.

You also need a pt file to detect the rubber cone.

## Prerequisites
It runs on Ubuntu 20.04, and the version below doesn't matter if you install the latest version.

- numpy
- open3d
- opencv
- pcl

## Topics
Before using this code, make sure you have pcl and serial_node.cpp, check the index on the webcam and write the camera index number on each left and right in web_cam_publish.py.


## Inference
Before you use this code, make sure you have pcl and serial_node.cpp.

```Shell
roscore
rosrun [catkin_ws/'your_directory'] lidar_cpp cone
rosrun [catkin_ws/'your_directory'] web_cam_publish.py
rosrun [catkin_ws/'your_directory'] multi_cam_4tracking.py
rosrun [catkin_ws/'your_directory'] calibration.py
```

If it doesn't run well, please look at the profile and contact us via "landsky1234@naver.com" email!

## Result
- https://www.youtube.com/watch?v=eNP2A6OB6Zw


