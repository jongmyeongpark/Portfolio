import os
import json

# JSON 파일이 있는 디렉토리 경로
input_dir = r"C:\Users\User\Desktop\AIHUB\라벨링데이터\1900_1200\daylight"
# 수정된 JSON 파일이 저장될 디렉토리 경로
output_dir = r"C:\Users\User\Desktop\AIHUB\output"

# 디렉토리가 존재하지 않으면 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 이미지 크기
image_height = 1200
image_width = 1900

for filename in os.listdir(input_dir):
    if filename.endswith('.json'):
        json_path = os.path.join(input_dir, filename)  # JSON 파일 경로
        
        with open(json_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        for annotation in data['annotations']:
            data_length = len(annotation['data'])
            # 기존 좌표에 대해 (x + 5, y + 5) 좌표 추가
            new_data = []
            for i in range(data_length):
                criterion = annotation['data'][i]
                x_tmp = criterion['x']
                y_tmp = criterion['y']
                if x_tmp >= 1920:
                    x_tmp = 1919
                else:
                    x_tmp = criterion['x'] + 5
                
                if y_tmp >= 1200:
                    y_tmp = 1199
                else:
                    y_tmp = criterion['y'] + 5
                new_data.append({"x": x_tmp, "y": y_tmp})
            # 기존 좌표에 추가된 좌표를 합침
            annotation['data'].extend(new_data)
        
        # 수정된 데이터를 저장할 JSON 파일 경로
        modified_json_path = os.path.join(output_dir, filename)
        
        with open(modified_json_path, 'w', encoding='utf-8') as file:
            json.dump(data, file, ensure_ascii=False, indent=4)

print("JSON 파일에 값 추가 완료!")
