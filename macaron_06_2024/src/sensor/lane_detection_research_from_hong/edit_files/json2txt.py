import os
import json

# JSON 파일이 있는 디렉토리 경로
input_dir = r"C:\Users\User\Desktop\AIHUB\output"
# 출력 txt 파일이 저장될 디렉토리 경로
output_dir = r"C:\Users\User\Desktop\AIHUB\output_2"

# 이미지 크기
image_height = 1200
image_width = 1920

# 디렉토리가 존재하지 않으면 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 클래스 ID 설정 (traffic_lane = 0으로 가정)
class_id = 0

# JSON 파일 처리
for filename in os.listdir(input_dir):
    if filename.endswith('.json'):
        json_path = os.path.join(input_dir, filename)
        
        with open(json_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        txt_filename = os.path.splitext(filename)[0] + '.txt'
        txt_path = os.path.join(output_dir, txt_filename)
        
        with open(txt_path, 'w', encoding='utf-8') as file:
            for annotation in data['annotations']:
                if annotation['class'] == 'traffic_lane':
                    coords = annotation['data']
                    normalized_coords = []
                    for coord in coords:
                        x_normalized = coord['x'] / image_width
                        y_normalized = coord['y'] / image_height
                        normalized_coords.append(f"{x_normalized:.6f}")
                        normalized_coords.append(f"{y_normalized:.6f}")
                    
                    coords_str = " ".join(normalized_coords)
                    file.write(f"{class_id} {coords_str}\n")

print("JSON 파일을 Ultralytics YOLO 형식의 txt 파일로 변환 완료!")
