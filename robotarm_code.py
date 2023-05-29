import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np
import cv2
import time

positions = []
use_color = []
mapped_value_x = 0
mapped_value_y = 0

cap = cv2.VideoCapture(2)
def opencv_start():
    #global x, y, color_name
    is_first_circle = True  # 첫 번째 원인지 여부를 나타내는 변수
    while True:
        # 프레임 읽기
        ret, frame = cap.read()
        # 프레임이 성공적으로 읽혔는지 확인
        if not ret:
            break
        # 필터링
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.equalizeHist(gray)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        #cv2.imshow("filter", edges)
        # 원 검출
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                param1=20,
                                param2=20,
                                minRadius=5,
                                maxRadius=12)
        if circles is not None:
            # 검출된 원의 중심과 반지름 추출
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # 반지름이 20 이하인 원을 초록색 원으로 그리기
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                cv2.circle(frame, (x, y), 15, (255, 0, 0), 2)
                cv2.circle(frame, (x, y), 40, (255, 0, 0), 2)
                # 반지름이 15와 40인 두 원 사이의 픽셀 색상 검출
                color = frame[y+20,x]  # 반지름 15인 원의 픽셀 색상
                b, g, r = color
                color_name = 'none'
                if g>b and g>r:
                #if b>170 and g>150 and r>150:
                    color_name = 'green'
                elif r>g and r>b and g<100:
                #elif b < 100 and g<100 and r>130:
                    color_name = 'red'
                elif b<g and b<r:
                #elif b<170 and g>200 and r>200:
                    color_name = 'yellow'
                # 반지름 40인 원의 픽셀 색상
                #print("Color 1:", color)
                cv2.putText(frame, f"{x, y} / {color_name} / {color}", (x - 50, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
                x, y, r = circles[0]
                if is_first_circle:
                    positions.append([x, y])  # 첫 번째 원의 좌표를 리스트에 추가
                    is_first_circle = False  # 첫 번째 원 처리가 끝났으므로 변수 업데이트

        print(positions)
        # 결과 영상 출력
        cv2.imshow("Circle Detection", frame)

        if len(positions) > 0:
            cap.release()
            cv2.destroyAllWindows()

def robot_start():
    # 유저 파라미터
    ip = "192.168.1.6"              # Robot의 IP 주소
    gripper_port = 1                # 그리퍼 포트 번호
    speed_value = 10                # 로봇 속도 (1~100 사이의 값 입력)

    # 로봇 연결
    dashboard, move, feed = connect_robot(ip)
    dashboard.EnableRobot()
    print("이제 로봇을 사용할 수 있습니다!")

    # 쓰레드 설정
    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    return move, gripper_port, dashboard


def connect_robot(ip):
    try:
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("연결 설정 중...")

        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print("연결 성공!!")

        return dashboard, move, feed

    except Exception as e:
        print("연결 실패")
        raise e
    
def robot_speed(dashboard : DobotApiDashboard, speed_value):
    dashboard.SpeedFactor(speed_value)
    
def gripper_DO(dashboard : DobotApiDashboard, index, status):
    dashboard.ToolDO(index, status)
    
def get_Pose(dashboard : DobotApiDashboard):
    dashboard.GetPose()
    
def run_point(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])
    
current_actual = None   # 전역 변수 (현재 좌표)

def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0

    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        a = np.frombuffer(data, dtype=MyType)

        if hex((a['test_value'][0])) == '0x123456789abcdef':
            current_actual = a["tool_vector_actual"][0]     # Refresh Properties
        sleep(0.001)

def x_axis_mapping(value_x):
    # 이전 범위에서의 값(value)을 새로운 범위로 맵핑하는 함수
    robot_range = 340 - 190
    camera_range = 432 - 60
    x_value = (((value_x - (60)) * robot_range*1.1) / camera_range) + 190
    return x_value

def y_axis_mapping(value_y):
    robot_range = 102 - (-125)
    camera_range = 536 - 30
    y_value = (((value_y - (30)) * robot_range) / camera_range) - 125
    return y_value

def move_robot(move):
    mapped_value_x = x_axis_mapping(positions[1])
    mapped_value_y = y_axis_mapping(positions[0])
    point_home = [mapped_value_x, mapped_value_y, -57, 0]
    run_point(move, point_home)

def move_Aruco(move, use_color):
    try:
        if len(use_color) == 1:
            if use_color[0] == 'green':
                point_home = [310, 390, -57, 0]
                run_point(move, point_home)
            elif use_color[0] == 'red':
                point_home = [60, 195, -57, 0]
                run_point(move, point_home) 
            else:
                point_home[0] = [545, 200, -57, 0]
                run_point(move, point_home)
        if len(use_color) == 2:
            if use_color[1] == 'green':
                point_home = [310, 390, -57, 0]
                run_point(move, point_home)
            elif use_color[1] == 'red':
                point_home = [60, 195, -57, 0]
                run_point(move, point_home) 
            else:
                point_home[1] = [545, 200, -57, 0]
                run_point(move, point_home)
        if len(use_color) == 3:
            if use_color[2] == 'green':
                point_home = [310, 390, -57, 0]
                run_point(move, point_home)
            elif use_color[2] == 'red':
                point_home = [60, 195, -57, 0]
                run_point(move, point_home) 
            else:
                point_home[2] = [545, 200, -57, 0]
                run_point(move, point_home)
    except:
        pass


def go_to_top(move,positions):
    if len(positions) == 1:
        mapped_value_x = x_axis_mapping(positions[0][1])
        mapped_value_y = y_axis_mapping(positions[0][0])
        point_home = [mapped_value_x, mapped_value_y, -24.5, 0]
        run_point(move, point_home)


def rearrange():
    pass


def main():
    opencv_start()
    move, gripper_port, dashboard = robot_start()
    # # gripper_DO(dashboard, gripper_port, 0)
    # gripper_DO(dashboard, gripper_port, 1)
    # cam_value = [545,200]
    # 빨간 qr = [60,195]
    # 그린 qr = [310,390]
    # 노랑 qr = [545,200]
    time.sleep(0.1)

    # 탑으로 가기
    go_to_top(move,positions)

    # # 위에꺼 집기
    # gripper_DO(dashboard, gripper_port, 1)
    # point_home = [mapped_value_x, mapped_value_y, -10, 0]
    # run_point(move, point_home)
    
    # # 아르꼬에 놓기 및 원위치
    # move_Aruco(move,use_color)
    # gripper_DO(dashboard, gripper_port, 0)
    # point_home = [284.57,0,121.2,0, 0] # 원위치
    # run_point(move, point_home)

    # # 다음 물건 정보 읽기 및 탑으로 위치
    # opencv_start()
    # go_to_top(move,positions)

    # # 두번째꺼 잡기
    # gripper_DO(dashboard, gripper_port, 1)
    # point_home = [mapped_value_x, mapped_value_y, -40, 0]
    # run_point(move, point_home)

    # # 아르꼬에 놓기 및 원위치
    # move_Aruco(move,use_color)
    # gripper_DO(dashboard, gripper_port, 0)
    # point_home = [284.57,0,121.2,0, 0] # 원위치
    # run_point(move, point_home)

    # # 다음 물건 정보 읽기 및 탑으로 위치
    # opencv_start()
    # go_to_top(move,positions)

    # # 마지막꺼 잡기
    # gripper_DO(dashboard, gripper_port, 1)
    # point_home = [mapped_value_x, mapped_value_y, -57, 0]
    # run_point(move, point_home)

    # # 아르꼬에 놓기 및 원위치
    # move_Aruco(move,use_color)
    # gripper_DO(dashboard, gripper_port, 0)
    # point_home = [284.57,0,121.2,0, 0] # 원위치
    # run_point(move, point_home)

    # ###### 모든 물체가 각 아르꼬에 위치해 있다. #######
    # if len(use_color) == 3:
    #     if use_color[2] == 'green':
    #         # 빨강 아르꼬로 이동 및 잡기
    #         point_home = [60, 195, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)
            
    #         # 초록 아르꼬에 놓기
    #         point_home = [310, 390, -40, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    #         # 노랑 아르꼬로 이동 및 잡기
    #         point_home = [545, 200, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)

    #         # 초록 아르꼬에 놓기
    #         point_home = [310, 390, -24.5, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    #     elif use_color[2] == 'red':
    #         # 초록 아르꼬로 이동 및 잡기
    #         point_home = [310, 390, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)

    #         # 빨강 아르꼬에 놓기
    #         point_home = [60, 195, -40, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    #         # 노랑 아르꼬로 이동 및 잡기
    #         point_home = [545, 200, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)

    #         # 빨강 아르꼬에 놓기
    #         point_home = [60, 195, -24.5, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    #     else:
    #         # 초록 아르꼬로 이동 및 잡기
    #         point_home = [310, 390, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)

    #         # 노랑 아르꼬에 놓기
    #         point_home = [545, 200, -40, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    #         # 빨강 아르꼬로 이동 및 잡기
    #         point_home = [60, 195, -57, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 1)
    #         go_to_top(move,positions)

    #         # 노랑 아르꼬에 놓기
    #         point_home = [545, 200, -24.5, 0]
    #         run_point(move, point_home)
    #         gripper_DO(dashboard, gripper_port, 0)
    #         go_to_top(move,positions)

    # 로봇 구동 1 (point_init)

    # gripper -57 1층 높이, gripper -24.5 3층 높이
    # point_home = [predicted_robot[0][0], predicted_robot[0][1], -24, 0]

    # wait_arrive(point_home)

# def main():
#     robot_start_thread = threading.Thread(target=robot_start)
#     opencv_start_thread = threading.Thread(target=opencv_start)
#     robot_move_thread = threading.Thread(target=move_robot)

#     robot_start_thread.start()
#     opencv_start_thread.start()
#     robot_move_thread.start()

    # # 쓰레드 종료 대기

    # thread1.join()
    # thread2.join()
if __name__ == "__main__":
    main()
