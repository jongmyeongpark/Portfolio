```lane_detection_research_from_hong
├── edit_files
│   ├── capture_3sec_videos2images.py
        - 동영상을 n초간 간격을 설정하여 이미지로 만들어 labeling할 이미지로 만듭니다.
│   ├── json2txt.py
        -  polyline2polygon파일을 실행한 후 yolov8의 label정보를 json에서 txt로 바꿀 수 있습니다.
│   └── polyline2polygon.py
        - Ai Hub에서 차선은 polyline입니다. yolov8은 polygon이기 때문에 polyline2polygon으로 바꿉니다.
├── pt
│   ├── Lane_Pretrained.pt
        - custom 데이터로 사전학습된 차선 yolov8s pt입니다.
│   ├── ai__hub__tt.pt
        - AI Hub에서 경기도 지역 내의 데이터로, 사전학습된 yolov8s pt입니다
│   ├── pretrained_traffic_cone_yolov8s.pt
        - 혹여나 하는 마음으로, rubber cone을 roboflow에서 받아와 사전학습된 yolov8s pt입니다.
│   ├── only_traffic_lane.pt
        - 7월23일 기준 class : traffic_lane만 있는 pt입니다.
│   ├── only_traffic_lane_2.pt
        - 8월 3일 기준 class : traffic_lane만 있으며, 기존에 있던 only_traffic_lane.pt보다 1,000장 더 학습시켰습니다.(약 6,000장)
└── intrinsic_parameters.txt
        - 차선 카메라의 내부 파라미터 정보입니다. (DarkProgrammer 사용)
└── segmentation_lane_detection.py
        - 8월 2일 기준 최종 차선검출 코드입니다. (중앙선, steer, 차선 곡률, heading과 왼/오른쪽 사이의 거리 추출 가능 및 실제 주행해보면서 발생한 문제들을 모두 예외처리 했습니다.)
└── training_for_lane_detection.ipynb
        - yolov8모델을 훈련시킬 수 있는 코드입니다.
``` 

### TOPIC
Publish node name: lane_detection

publish topics

* steer angle: steer를 얼마만큼 줘야할지(?), radian의 값으로 보내줍니다.(type : Float64)

* left_heading_distance_pub: heading과 왼쪽 차선 사이의 거리

* right_heading_distance_pub: heading과 오른쪽 차선 사이의 거리
