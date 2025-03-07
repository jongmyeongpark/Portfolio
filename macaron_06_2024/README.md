# macaron_6

usb port : imu -> erp -> gps

Macaron 코드 순서

USB 순서
IMU -- GPS — ERP 순서

//usb포트 권한부여
usb0 > usb1 > acm0
((or USB))

//Gps 키기 —  포트37 – 리시버 - 엔트리클라이언트 - 업데이트 소스 테이블
roslaunch macaron_06 sensor_ready.launch

//차랑 연결
roslaunch macaron_06 connect car.launch

//state 노드 실행
rosrun macaron_06 state.py

//비쥬얼 실행
roslaunch macaron_06 visualization.launch

//매핑하기
//mapping.py에서 경로 이름 바꿔주고, 아래 명령어 실행
//0.5m간격으로 점 찍음 
rosrun macaron_06 mapping.py


catkin_make가 안될시 -> velodyne driver를 먼저 cmake한 이후에 cmake하면 된다.
