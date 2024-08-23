import torch
from pathlib import Path
import cv2
from matplotlib import pyplot as plt

# 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/macaron/park/helmet/best.pt')
#/home/park/catkin_ws/src/Helmet-Detection/weights/best.pt


# 이미지 경로 설정
img_path = 'helmet/helmet_jm.jpg'
#/home/park/Downloads/helmet_jm.jpg

# 이미지 로드
img = cv2.imread(img_path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# 추론 수행
results = model(img)

# 결과 출력
results.show()

# 결과를 파일로 저장
results.save('output')


