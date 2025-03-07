# Shark MORAI Competition
I won the Excellence Award at the Shark MORAI competition, and it is the code for the preliminary and final matches.
![image](https://github.com/user-attachments/assets/b1729515-7392-4749-bba4-82bca60295ed)


## Introduction
A total of 100 students participated, and 신희승, 홍성민, 박종명, 조영우, 권세빈, 이승진 participated in the MACARON team of Dongguk University.

For more information, please check out the news.
[MORAI 디지털트윈 성남시 기반 SHARK 자율주행대회 성료](https://www.morai.ai/ko/post/%EB%AA%A8%EB%9D%BC%EC%9D%B4-%EB%94%94%EC%A7%80%ED%84%B8%ED%8A%B8%EC%9C%88-%EC%84%B1%EB%82%A8%EC%8B%9C-%EA%B8%B0%EB%B0%98-%EC%83%A4%ED%81%AC%EC%9E%90%EC%9C%A8%EC%A3%BC%ED%96%89%EB%8C%80%ED%9A%8C-%EC%84%B1%EB%A3%8C)

## Prerequisites
It runs on Ubuntu 20.04, and the version below doesn't matter if you install the latest version.

Because SLAM was used in the GPS shaded section, FAST_LIO2 must be installed.

- numpy
- open3d
- opencv
- ultralytics
- FAST_LIO2(SLAM Pacakage)
- morai_msgs
- velodyne_driver

## Inference

```Shell
roscore
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch [catkin_ws/'your_directory'] sensor_ready.launch
rosrun [catkin_ws/'your_directory'] state.py
rosrun [catkin_ws/'your_directory'] tracking_planner.py
roslaunch fast_lio mapping_velodyne_morai_launch
rosrun [catkin_ws/'your_directory'] mission_state.py or rosrun [catkin_ws/'your_directory'] mission_state_bon.py
```

If it doesn't run well, please look at the profile and contact us via "landsky1234@naver.com" email!

## Scenario
[Download from GoogleDrive](https://drive.google.com/drive/folders/1sBTfBEsG0oSAXzzPXgU61zWrStDhcfgR?usp=sharing)


## Result
- A Preliminary Round: https://www.youtube.com/watch?v=3tq5JOpf_dU
- A Final Round: https://www.youtube.com/watch?v=oIO3mFCJtt0

