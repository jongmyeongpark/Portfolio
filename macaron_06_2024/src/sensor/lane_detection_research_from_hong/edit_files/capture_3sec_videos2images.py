import cv2
import os

def capture_frames(video_path, output_folder, interval=0.5, width=640, height=480):
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"Error: Cannot open video file {video_path}")
        return
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    frame_interval = int(fps * interval)
    
    os.makedirs(output_folder, exist_ok=True)
    
    frame_count = 0
    img_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        if frame_count % frame_interval == 0:
            # 이미지 크기 조정
            resized_frame = cv2.resize(frame, (width, height))
            img_filename = os.path.join(output_folder, f"yotube_6_frame_{img_count:04d}.png")
            cv2.imwrite(img_filename, resized_frame)
            print(f"Saved {img_filename}")
            img_count += 1
        
        frame_count += 1
    
    cap.release()
    print("Done capturing frames.")

video_path = '2024-07-01-135042.webm'
output_folder = 'kookmin_autonomous_3.dataset'

capture_frames(video_path, output_folder, interval=1, width=640, height=480)