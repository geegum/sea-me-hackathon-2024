#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Float32

# ROS 노드 초기화
rospy.init_node('line_follower', anonymous=True)

# Publisher 생성
steering_publisher = rospy.Publisher('/Steering', Float32, queue_size=10)
throttle_publisher = rospy.Publisher('/Throttle', Float32, queue_size=10)

# 나머지 변수 및 함수 정의는 그대로 유지
blk_size = 9        # 블럭 사이즈
C = 5               # 차감 상수 

# (wvXXX, wvXXXvalue, wvR1, wvR, wvL1, wvL, wvX, wvY, wvxxxvaluevalue 정의 유지)

cntGo = 0
cntRight = 0
cntLeft = 0

gijun = 30000
cntgijun = 1

def nothing(x):
    pass

# settingGoal_bar 함수 유지

settingGoal_bar()

cap = cv2.VideoCapture(0)               # 0번 카메라 장치 연결
if cap.isOpened():                      # 캡쳐 객체 연결 확인
    stdTimeE = time.time()  # 초기 시간 설정
    while not rospy.is_shutdown():
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

        # ROS 메시지 발행
        steering_publisher.publish(Float32(lineDetect))
        throttle_publisher.publish(Float32(Throttle))

        cv2.imshow('camera', img)
        cv2.imshow('th3', line_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

else:
    print("can't open camera.")

cap.release()  # 자원 반납
cv2.destroyAllWindows()
