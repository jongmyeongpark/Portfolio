import cv2

# 비디오 장치 접근
cap = cv2.VideoCapture('/dev/video2')

# 영상이 성공적으로 열렸는지 확인
if not cap.isOpened():
    print("Error: Could not open video stream from /dev/video2.")
    exit()

while True:
    ret, frame = cap.read()  # 프레임을 읽어옵니다.
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # 프레임을 윈도우에 표시합니다.
    cv2.imshow('DroidCam Live Stream', frame)
    
    # 'q' 키를 누르면 루프를 탈출합니다.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스를 해제합니다.
cap.release()
cv2.destroyAllWindows()
