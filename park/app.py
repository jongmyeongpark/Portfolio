from flask import Flask, Response, jsonify, render_template_string
import cv2
import torch
import base64

app = Flask(__name__)

# YOLOv5 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/macaron/park/helmet/best.pt')

def generate_frames():
    cap = cv2.VideoCapture('/dev/video2')  # DroidCam 비디오 장치

    while True:
        success, frame = cap.read()  # 프레임 읽기
        if not success:
            break
        
        # YOLOv5로 헬멧 탐지 수행
        results = model(frame)
        results.render()  # 결과 렌더링
        
        # 확률 체크
        helmet_detected = False
        for det in results.pred[0]:  # 각 탐지된 객체에 대해
            if det[-1] == 1:  # 클래스 1이 헬멧인 경우
                confidence = det[4].item()  # 신뢰도 값
                if confidence >= 0.9:
                    helmet_detected = True
                    break
        
        # 메시지 설정
        message = "헬멧을 착용했습니다" if helmet_detected else "헬멧을 착용해주세요"
        
        # 렌더링된 이미지를 다시 frame에 저장
        for img in results.ims:
            frame = img

        # 프레임을 JPEG 형식으로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = base64.b64encode(buffer).decode('utf-8')
        
        # 프레임과 메시지를 JSON 형태로 반환
        yield f'data: {{"frame": "{frame}", "message": "{message}"}}\n\n'

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='text/event-stream')

@app.route('/')
def index():
    return render_template_string('''
    <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>삐용삐용 - 헬멧 착용 시스템</title>
            <style>
                body {
                    font-family: 'Arial', sans-serif;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                    background: linear-gradient(135deg, #e8f0f2, #ffffff);
                    color: #333;
                }
                .container {
                    text-align: center;
                    background-color: #ffffff;
                    border-radius: 12px;
                    padding: 40px;
                    box-shadow: 0 6px 12px rgba(0, 0, 0, 0.1);
                    width: 90%;
                    max-width: 600px;
                    transition: transform 0.3s ease;
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                }
                .container:hover {
                    transform: scale(1.02);
                }
                h1 {
                    color: #ff6f61;
                    margin-bottom: 20px;
                    font-size: 2.5em;
                    font-family: 'Comic Sans MS', cursive, sans-serif;
                }
                p {
                    font-size: 1.4em;
                    color: #555;
                    margin-top: 0;
                }
                #message {
                    font-weight: bold;
                    color: #ff5722;
                    font-size: 1.5em;
                    text-shadow: 1px 1px 3px rgba(0, 0, 0, 0.2);
                    margin-top: 20px;
                }
                .footer {
                    width: 100%;
                    text-align: center;
                    font-size: 0.9em;
                    color: #888;
                    font-family: 'Comic Sans MS', cursive, sans-serif;
                    margin-top: 20px;
                }
                .button {
                    margin-top: 20px;
                    padding: 10px 20px;
                    font-size: 1em;
                    color: #fff;
                    background-color: #ff6f61;
                    border: none;
                    border-radius: 5px;
                    cursor: pointer;
                    transition: background-color 0.3s ease;
                }
                .button:hover {
                    background-color: #e64a19;
                }
                #video {
                    width: 600px;
                    height: auto;
                    border-radius: 10px;
                    margin-top: 20px;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1><img src="https://cdn-icons-png.flaticon.com/512/819/819540.png" height="40px" width="40px"> 헬멧 감지 <img src="https://cdn-icons-png.flaticon.com/512/819/819540.png" height="40px" width="40px"></h1>
                <p>실시간 헬멧 착용 상태:</p>
                <img id="video">
                <div id="message"></div>
                <button class="button" onclick="refreshData()">새로 고침</button>
            </div>

            <script>
                const video = document.getElementById('video');
                const message = document.getElementById('message');

                const evtSource = new EventSource("/video_feed");

                evtSource.onmessage = function(event) {
                    const data = JSON.parse(event.data);
                    video.src = 'data:image/jpeg;base64,' + data.frame;
                    message.textContent = data.message;
                };

                function refreshData() {
                    evtSource.onmessage();
                }
            </script>
        </body>
    </html>
    ''')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)




###일단 잘되는 버전 ###
# from flask import Flask, Response, jsonify, render_template_string
# import cv2
# import torch
# import base64

# app = Flask(__name__)

# # YOLOv5 모델 로드
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/macaron/park/helmet/best.pt')

# def generate_frames():
#     cap = cv2.VideoCapture('/dev/video2')  # DroidCam 비디오 장치

#     while True:
#         success, frame = cap.read()  # 프레임 읽기
#         if not success:
#             break
        
#         # YOLOv5로 헬멧 탐지 수행
#         results = model(frame)
#         # print(results)
#         results.render()  # 결과 렌더링
        
#         # 확률 체크
#         helmet_detected = False
#         for det in results.pred[0]:  # 각 탐지된 객체에 대해
#             # print(det)
#             if det[-1] == 1:  # 클래스 0 이면 머리인듯??  1이 헬멧인듯 

#                 confidence = det[4].item()  # 신뢰도 값
#                 print(confidence)
#                 if confidence >= 0.9:
#                     helmet_detected = True
#                     break
        
#         # 메시지 설정
#         message = "헬멧을 착용했습니다" if helmet_detected else "헬멧을 착용해주세요"
        
#         # 렌더링된 이미지를 다시 frame에 저장
#         for img in results.ims:  # results.ims는 렌더링된 이미지를 포함
#             frame = img

#         # 프레임을 JPEG 형식으로 인코딩
#         ret, buffer = cv2.imencode('.jpg', frame)
#         frame = base64.b64encode(buffer).decode('utf-8')
        
#         # 프레임과 메시지를 JSON 형태로 반환
#         yield f'data: {{"frame": "{frame}", "message": "{message}"}}\n\n'

# @app.route('/video_feed')
# def video_feed():
#     return Response(generate_frames(), mimetype='text/event-stream')

# @app.route('/')
# def index():
#     return render_template_string('''
#     <html>
#         <head>
#             <title>삐용삐용</title>
#         </head>
#         <body>
#             <h1>헬멧 착용 결과 화면</h1>
#             <img id="video" width="640" height="480">
#             <h2 id="message"></h2>
#             <script>
#                 const video = document.getElementById('video');
#                 const message = document.getElementById('message');

#                 const evtSource = new EventSource("/video_feed");

#                 evtSource.onmessage = function(event) {
#                     const data = JSON.parse(event.data);
#                     video.src = 'data:image/jpeg;base64,' + data.frame;
#                     message.textContent = data.message;
#                 }
#             </script>
#         </body>
#     </html>
#     ''')

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000)



### 초기버전 ###
# from flask import Flask, Response
# import cv2
# import torch

# app = Flask(__name__)

# # YOLOv5 모델 로드
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/macaron/park/helmet/best.pt')

# def generate_frames():
#     cap = cv2.VideoCapture('/dev/video2')  # DroidCam 비디오 장치

#     while True:
#         success, frame = cap.read()  # 프레임 읽기
#         if not success:
#             break
        
#         # YOLOv5로 헬멧 탐지 수행
#         results = model(frame)
#         results.render()  # 결과 렌더링
        
#         # 렌더링된 이미지를 다시 frame에 저장
#         for img in results.ims:  # results.ims는 렌더링된 이미지를 포함
#             frame = img

#         # 프레임을 JPEG 형식으로 인코딩
#         ret, buffer = cv2.imencode('.jpg', frame)
#         frame = buffer.tobytes()
        
#         # 프레임을 바이트 형식으로 반환
#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# @app.route('/video_feed')
# def video_feed():
#     # /video_feed로 요청이 오면 실시간 스트림을 반환
#     return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/')
# def index():
#     # 기본 웹 페이지
#     return '''
#     <html>
#         <head>
#             <title>Helmet Detection</title>
#         </head>
#         <body>
#             <h1>Helmet 착용 여부</h1>
#             <img src="/video_feed" width="640" height="480">
#         </body>
#     </html>
#     '''

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000)
