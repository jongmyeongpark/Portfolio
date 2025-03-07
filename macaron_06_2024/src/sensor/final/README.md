### VISION, 본선 코드들 입니다.

```traffic
└── traffic_combined.py
        - 이미지 2개를 합쳐서 신호등 인지하는 코드, 참고용

└── traffic_delivery_pt2.py
        - 배달표지판, 신호등을 인지하는 코드
        - pt파일 2개 (배달표지판, 신호등)

├── lidar_4delivery.cpp
        - 본선 배달 미션용 lidar 코드

├── calibration_4delivery.py
        - 본선 배달 미션용 calibration 코드 

└── multi_cam_4final.py
        - 배달표지판, 신호등을 인지하는 코드
        - pt파일 1개 (배달표지판 + 신호등)

└── mission_traffic_fmtc_with_gps.py
        - GPS 정보를 바탕으로 신호등 Go, Stop 정보 알려주는 코드

```
### 배달 미션용
1. rosrun macaron_06 multi_cam_4final.py
2. rosrun macaron_06 lidar_4delivery.cpp
3. rosrun macaron_06 calibration_4delivery.py


### 신호등
1. rosrun macaron_06 multi_cam_4final.py
2. rosrun macaron_06 mission_traffic_fmtc_with_gps.py

