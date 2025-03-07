```tracking
├── traffic_cone_yolov8s.pt
        - rubber cone(class: yellow_cone, blue_cone)을 학습한 yolov8 모델입니다.

├── fix_calibration_parameters.py
        - sensor fusion시, translate/rotation parameters를 맞추는 코드입니다. 

└── lidar.cpp
        - lidar의 pointcloud를 "전처리"만 하는 코드입니다.

└── web_cam_publish.py
        - 왼/오른쪽 카메라 이미지 토픽을 publish해주는 코드입니다.

├── calibration_4tracking.py
        - lidar, object detection의 결과물을 받고 콘의 x,y좌표를 보내주며 z좌표에는 파란콘은 0, 노란콘은 1을 담아서 보내주는 코드입니다.

└── multi_cam_4tracking.py
        - 2개의 카메라 이미지 정보를 받아 왼쪽, 오른쪽의 rubber cone을 detection하는 코드입니다.

└── mission_4tracking.py
        - 최종 tracking 코드입니다.

```
### Track MISSION 코드 키는 순서 
1. rosrun macaron_06 lidar_cpp
2. rosrun macaron_06 web_cam_publish.py
3. rosrun macaron_06 multi_cam_4tracking.py
4. rosrun macaron_06 calibration_4tracking.py
5. rosrun macaron_06 mission_4tracking.py
   
**roslaunch macaron_06 mission_track.launch**, 한번에 실행 가능
