# 🪖 Capstone Design — Helmet Detection

> Capstone Design Competition (Encouragement Award), 2024.07 ~ 2024.08

A real-time **helmet-wearing detection** system built with a **YOLOv5** custom model and served through a **Flask** web app. A webcam stream (DroidCam) is run through the detector, and the UI tells the rider whether a helmet is being worn — intended as a safety check for personal mobility (e.g. kickboard / e-bike) rental.

## 🔍 How it works

1. `web_cam` (DroidCam via `/dev/video2`) provides the live video stream.
2. A custom-trained YOLOv5 model (`helmet/best.pt`) detects the *helmet* class per frame.
3. If a helmet is detected with confidence ≥ 0.9, the app shows **"헬멧을 착용했습니다"**, otherwise **"헬멧을 착용해주세요"**.
4. An Arduino (serial, `/dev/ttyUSB0`) is used for hardware feedback / gate control.

## 📂 Key Files

| File | Description |
| ---- | ----------- |
| `app.py` | Flask server — streams webcam frames with YOLOv5 helmet detection overlay |
| `arduino.py` | Streamlit UI + serial communication with Arduino (`/dev/ttyUSB0`, 9600 baud) |
| `helmet0714.py` | Standalone single-image helmet inference (YOLOv5) |
| `show_droid.py` | DroidCam stream viewer utility |
| `helmet/best.pt` | Custom-trained YOLOv5 weights |
| `templates/`, `static/`, `*.html` | Web UI |

## 🛠️ Stack

- **Python**, **Flask**, **Streamlit**
- **YOLOv5** (`torch.hub`, custom weights), **OpenCV**
- **Arduino** (`pyserial`) for hardware integration

## ▶️ Run

```shell
python app.py        # Flask web server with live helmet detection
# open the served page in a browser to view the annotated stream
```

> Note: device paths (`/dev/video2`, `/dev/ttyUSB0`) and the weights path in `app.py` may need to be adjusted for your machine.
