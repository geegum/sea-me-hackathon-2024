import cv2
import numpy as np
import time
import roslibpy
##################for ros to do socket tongsin
#sudo apt-get install ros-melodic-rosbridge-suite
#pip install roslibpy

#to start 
#roslaunch rosbridge_server rosbridge_websocket.launch
#python3 srcircmarathn.py
#


client = roslibpy.Ros(host='localhost', port=9090)
client.run()

steering_publisher = roslibpy.Topic(client, '/Steering', 'std_msgs/Float32')
throttle_publisher = roslibpy.Topic(client, '/Throttle', 'std_msgs/Float32')
###############
blk_size = 9        # 블럭 사이즈
C = 5               # 차감 상수 

wvXXX = np.ones((1,300))
wvXXXvalue = wvXXX * 150

wvR1 = np.arange(1,151) 
wvR = wvR1 / 3
wvL1 = np.flip(wvR)
wvL = wvL1 / 3 

wvX = np.arange(0,300) 
wvY = wvX.reshape(300,1)

wvxxxvaluevalue = (wvX - wvXXXvalue) / 3

cntGo = 0
cntRight = 0
cntLeft = 0

gijun = 30000
cntgijun = 1

port = "COM1"
baud = 115200

# ser = serial.Serial(port, baud)

def nothing(x):
    pass

# 목표지점 HSV찾는 토글바 열기
def settingGoal_bar():
    cv2.namedWindow('HSV_settings')
    cv2.resizeWindow('HSV_settings', 400, 250)

    cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 180, nothing)
    cv2.setTrackbarPos('H_MAX', 'HSV_settings', 180)
    cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 180, nothing)
    cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)

    cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('S_MAX', 'HSV_settings', 10)
    cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)

    cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('V_MIN', 'HSV_settings', 253)

settingGoal_bar()

cap = cv2.VideoCapture(0)               # 0번 카메라 장치 연결
if cap.isOpened():                      # 캡쳐 객체 연결 확인
    stdTimeE = time.time()  # 초기 시간 설정
    while True:
        H_max = cv2.getTrackbarPos('H_MAX', 'HSV_settings')
        H_min = cv2.getTrackbarPos('H_MIN', 'HSV_settings')
        S_max = cv2.getTrackbarPos('S_MAX', 'HSV_settings')
        S_min = cv2.getTrackbarPos('S_MIN', 'HSV_settings')
        V_max = cv2.getTrackbarPos('V_MAX', 'HSV_settings')
        V_min = cv2.getTrackbarPos('V_MIN', 'HSV_settings')

        ret, orginimg = cap.read()           # 다음 프레임 읽기
        if not ret:
            print('no frame')
            break

        img = cv2.resize(orginimg, (300, 300))
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        line_lower = np.array([H_min, S_min, V_min], np.uint8)
        line_upper = np.array([H_max, S_max, V_max], np.uint8)
        line_mask = cv2.inRange(hsvFrame, line_lower, line_upper)

        res_line = cv2.bitwise_and(img, img, mask=line_mask)

        thresh = line_mask
        thresh_02 = thresh / 255
        thresh_01 = thresh_02 * wvY
        threshrowsum1 = np.sum(thresh_01, axis=0)
        threshrowsum = threshrowsum1 * wvxxxvaluevalue * wvxxxvaluevalue * wvxxxvaluevalue
        threshallsum = (np.sum(threshrowsum, axis=1)) / 1000000
        lineDetect = int(threshallsum)
        
        
        
        # Throttle 계산
        if abs(lineDetect) < gijun:
            Throttle = 0.4 - (abs(lineDetect) / gijun) * 0.2  # 1에서 0.2까지 비례적으로 감소
        else:
            Throttle = 0.2
        
        # Throttle 범위 확인
        Throttle = max(0.2, min(1.0, Throttle))

        
        
        steering_publisher.publish(roslibpy.Message({'Steer': lineDetect}))
        throttle_publisher.publish(roslibpy.Message({'Throt': Throttle}))


        cv2.imshow('camera', img)
        cv2.imshow('th3', line_mask)

        if cv2.waitKey(1) != -1:
            break
else:
    print("can't open camera.")

cap.release()  # 자원 반납
cv2.destroyAllWindows()

client.terminate()
