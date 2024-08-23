import streamlit as st
import serial
import time

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyUSB0', 9600)

# HTML 스타일 적용
st.markdown("""
    <style>
        body {
            font-family: 'Arial', sans-serif;
            background: linear-gradient(135deg, #e8f0f2, #ffffff);
            color: #333;
        }
        .container {
            text-align: center;
            background-color: #ffffff;
            border-radius: 12px;
            padding: 30px;
            box-shadow: 0 6px 12px rgba(0, 0, 0, 0.1);
            width: 90%;
            max-width: 400px;
            margin: auto;
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
        #alcohol-value {
            font-weight: bold;
            color: #ff5722;
            font-size: 3em;
            text-shadow: 1px 1px 3px rgba(0, 0, 0, 0.2);
        }
        #status-message {
            margin-top: 20px;
            font-size: 1.2em;
            display: none;
        }
        .footer {
            position: fixed;
            bottom: 20px;
            left: 50%;
            transform: translateX(-50%);
            font-size: 0.9em;
            color: #888;
            font-family: 'Comic Sans MS', cursive, sans-serif;
        }
        .info-box {
            margin-top: 20px;
            padding: 15px;
            background-color: #ffebee;
            border: 1px solid #fdd;
            border-radius: 8px;
            font-size: 1em;
            color: #333;
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
    </style>
""", unsafe_allow_html=True)

# 메인 루프
placeholder = st.empty()

while True:
    sensor_value, ppm_value = None, None
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        try:
            sensor_value, ppm_value = line.split(",")
            sensor_value = float(sensor_value)
            ppm_value = float(ppm_value)
        except ValueError:
            pass

    with placeholder.container():
        st.markdown("<div class='container'>", unsafe_allow_html=True)
        st.markdown("<h1>삐용삐용</h1>", unsafe_allow_html=True)
        if sensor_value is not None and ppm_value is not None:
            st.markdown(f"<p>현재 측정값 : <span id='alcohol-value'>{ppm_value}</span></p>", unsafe_allow_html=True)

            thresholdPPM = 0.03 * 2100  # 0.03% 혈중 알코올 농도에 해당하는 PPM 값

            if ppm_value > thresholdPPM:
                st.markdown("<div id='status-message' style='color: #ff0000; display: block;'>경고: 혈중 알코올 농도가 임계치를 초과하였습니다!</div>", unsafe_allow_html=True)
            else:
                st.markdown("<div id='status-message' style='color: #4caf50; display: block;'>안전: 혈중 알코올 농도가 임계치 이하입니다.</div>", unsafe_allow_html=True)
        else:
            st.markdown("<p>데이터를 기다리고 있습니다...</p>", unsafe_allow_html=True)

        st.markdown("<div class='info-box'><p><strong>안내</strong> 이 시스템은 음주 여부를 측정합니다.</p></div>", unsafe_allow_html=True)
        st.markdown("</div>", unsafe_allow_html=True)
        
    time.sleep(1)
