import numpy as np
import matplotlib.pyplot as plt

class State:
    def __init__(self):
        self.L = 3  # 차축 간 거리
        self.x = 0.0  # 초기 x 위치
        self.y = 0.0  # 초기 y 위치
        self.theta = 0  # 초기 방향 (뒤로 후진)
        self.speed = -1.0  # 후진 속도 (음수)

    def update(self, x, y, heading):
        # 차량의 위치 및 방향 업데이트
        self.x = x
        self.y = y
        self.theta = heading

def pure_pursuit(target, vehicle: State):
    # 목표 위치와 차량의 현재 위치 간의 거리 계산
    dx = target[0] - vehicle.x
    dy = target[1] - vehicle.y
    distance = np.hypot(dx, dy)

    # 목표 방향 계산
    target_angle = np.arctan2(dy, dx)
    if target_angle < 0:
        target_angle += 2 * np.pi

    # print(target_angle, vehicle.theta)
    # 조향 각도 계산
    delta = target_angle - vehicle.theta
    if delta >= np.pi:
        delta = 2*np.pi - delta
    
    if delta <= -np.pi:
        delta = 2*np.pi + delta
    # print(delta)

    # 조향 각도를 제한
    max_steering_angle = np.deg2rad(37)  # 최대 조향 각도
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)

    return delta

class PurePursuitParking:
    def __init__(self):
        self.state = State()


    # 함수를 사용할 때 velocity 위치에 ld를 넣는데 이유가 뭐지..?
    def get_steer_state(self, x, y, heading, goal):
        self.state.update(x, y, heading)
        
        return pure_pursuit(goal, self.state)


# # 차량 및 경로 설정
# vehicle = State()
# target_path = np.array([[0, 0], [5, 5], [10, 0], [5, -5], [0, 0]])  # 목표 경로 설정

# # 시뮬레이션 설정
# trajectory_x = []
# trajectory_y = []

# for target in target_path:
#     while True:
#         # 조향 각도 계산
#         delta = pure_pursuit(target, vehicle)
        
#         # 차량 상태 업데이트
#         vehicle.update(delta)

#         # 기록
#         trajectory_x.append(vehicle.x)
#         trajectory_y.append(vehicle.y)

#         # 목표 위치에 도달했는지 확인
#         if np.hypot(vehicle.x - target[0], vehicle.y - target[1]) < 0.1:
#             break