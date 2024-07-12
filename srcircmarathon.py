# 카메라(웹캠) 프레임 읽기 (video_cam.py)
import serial
import cv2
import numpy as np
import time

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

gijun = 10000
cntgijun = 10

port = "COM1"
baud = 115200

#ser = serial.Serial(port,baud)


def nothing(x):
    pass
# 목표지점 HSV찾는 토글바 열기
def settingGoal_bar():
    cv2.namedWindow('HSV_settings')
    cv2.resizeWindow('HSV_settings',400,250)

    cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 180, nothing)
    cv2.setTrackbarPos('H_MAX', 'HSV_settings', 20)
    cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 180, nothing)
    cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)

    cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('S_MAX', 'HSV_settings', 20)
    cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)

    cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, nothing)
    cv2.setTrackbarPos('V_MIN', 'HSV_settings', 253)

settingGoal_bar()




cap = cv2.VideoCapture(0)               # 0번 카메라 장치 연결 ---①
if cap.isOpened():                      # 캡쳐 객체 연결 확인
    while True:
        H_max = cv2.getTrackbarPos('H_MAX', 'HSV_settings')
        H_min = cv2.getTrackbarPos('H_MIN', 'HSV_settings')
        S_max = cv2.getTrackbarPos('S_MAX', 'HSV_settings')
        S_min = cv2.getTrackbarPos('S_MIN', 'HSV_settings')
        V_max = cv2.getTrackbarPos('V_MAX', 'HSV_settings')
        V_min = cv2.getTrackbarPos('V_MIN', 'HSV_settings')
        
        
        
        
        
        
        ret, orginimg = cap.read()           # 다음 프레임 읽기
        img = cv2.resize(orginimg, (300, 300))
        
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        #img2 = img.astype(np.uint16)                # dtype 변경 ---①
        #b,g,r = cv2.split(img2)                     # 채널 별로 분리 ---②
        #b,g,r = img2[:,:,0], img2[:,:,1], img2[:,:,2]
        #gray1 = ((b + g + r)/3).astype(np.uint8)    # 평균 값 연산후 dtype 변경 ---③
        
        #gray2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # BGR을 그레이 스케일로 변경 ---④
        
        # --- ① NumPy API로 바이너리 이미지 만들기
        #thresh_np = np.zeros_like(gray2)   # 원본과 동일한 크기의 0으로 채워진 이미지
        #thresh_np[ gray2 < 127] = 255      # 127 보다 큰 값만 255로 변경
        # ---② OpenCV API로 바이너리 이미지 만들기
        #ret, thresh_cv = cv2.threshold(gray2, 127, 255, cv2.THRESH_BINARY) 
        
        
        #토츠 쓰레시홀드처리 적응형 임계값 적용 => 오픈씨브이에서 기본으로 제공하는 바이너리, 이진화보다 더 깔끔한 검출, 일일이 임계값을 찾지 않아도 됨.
        #t, t_otsu = cv2.threshold(gray2, -1, 255,  cv2.THRESH_BINARY | cv2.THRESH_OTSU) 
        
        #가우시안블러쓰레시홀드처리 
        #th3 = cv2.adaptiveThreshold(gray2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
        #                             cv2.THRESH_BINARY, blk_size, C)
        
        
        line_lower = np.array([H_min, S_min, V_min], np.uint8) 
        line_upper = np.array([H_max, S_max, V_max], np.uint8) 
        line_mask = cv2.inRange(hsvFrame, line_lower, line_upper) 
        
        res_line = cv2.bitwise_and(img, img, mask = line_mask)
        #print(line_mask)
        
        ########### 이진화 한 값을 0과 255를 바꿈 (그... 지금 하고 있는곳이 하얀색 배경에 검정색 그림이기 때문)
        thresh = line_mask
        ########너무 큰 숫자를 만들지 않기 위해 0 255를 0 1 로 바꿈 (행렬구조임)
        thresh_02 = thresh/255
        thresh_01 = thresh_02 * wvY 
        ######### 이 행렬구조의 원소들의 합을 구함
        ###########열을 다해서 행으로 만듦
        threshrowsum1 = np.sum(thresh_01, axis = 0)
        
        threshrowsum = threshrowsum1 * wvxxxvaluevalue * wvxxxvaluevalue* wvxxxvaluevalue
        
        ##############그 열안에 있는 원소들의 합을 다 함.
        threshallsum = (np.sum(threshrowsum, axis=1))/1000000
        
        
        
        '''
        ##### 반갈라서 왼쪽의 합을 구함
        leftthress = thresh_01[0:300, 0:150]
        leftthressrowsum = np.sum(leftthress, axis=0)
        leftthressrowsumWv = leftthressrowsum * wvL * wvL
        leftthresssumsum = np.sum(leftthressrowsumWv, axis= 0)
        
        ###############반갈라서 오른쪽 합을 구함
        rightthress = thresh_01[0:300, 150:300]
        rightthressrowsum = np.sum(rightthress, axis=0)
        rightthressrowsumWv = rightthressrowsum * wvR * wvR
        rightthresssumsum = np.sum(rightthressrowsumWv,axis=0)
        
        ####################150칸짜리 가중치를 위한 열을 만듦
        '''
        lineDetect =  int(threshallsum)
        
        
        #print(int(threshallsum) )
        
        #print(rightthresssumsum + leftthresssumsum)
        
        #헤리스 코너 응답함ㅅ 계산
        # 좋은 특징점 검출 방법
        #corners = cv2.goodFeaturesToTrack(t_otsu, 400, 0.1, 10)

        #dst1 = cv2.cvtColor(t_otsu, cv2.COLOR_GRAY2BGR)
        '''
        if corners is not None:
            for i in range(corners.shape[0]): # 코너 갯수만큼 반복문
                pt = (int(corners[i, 0, 0]), int(corners[i, 0, 1])) # x, y 좌표 받아오기
                cv2.circle(dst1, pt, 5, (0, 0, 255), 2) # 받아온 위치에 원
       
        
        #fast 코너 검출
        fast = cv2.FastFeatureDetector_create(60) # 임계값 60 지정
        keypoints = fast.detect(gray2) # Keypoint 객체를 리스트로 받음

        dst2 = cv2.cvtColor(gray2, cv2.COLOR_GRAY2BGR)

        for kp in keypoints:
            pt = (int(kp.pt[0]), int(kp.pt[1])) # kp안에 pt좌표가 있음
            cv2.circle(dst2, pt, 5, (0, 0, 255), 2)
        
        '''
        
        if (-gijun < lineDetect < gijun):
            cntGo = cntGo+1
            
        elif(lineDetect > gijun):
            cntRight = cntRight+1
        elif(lineDetect < -gijun):
            cntLeft = cntLeft + 1
        
        stdTimeS = time.time()
        if ((stdTimeS - stdTimeE)>=2):
            if (cntGo > cntgijun ):
                #ser.write(b'JK0002;')
                time.sleep(0.01)

                print("gogogo")
                cntGo = 0
                cntLeft = 0
                cntRight = 0
            elif(cntLeft > cntgijun):
                #ser.write(b'JK0080;')
                time.sleep(0.01)
                print("left")
                cntGo = 0
                cntLeft = 0
                cntRight = 0
            elif (cntRight > cntgijun):
                #ser.write(b'JK0008;')
                time.sleep(0.01)
                print("right")
                cntGo = 0
                cntLeft = 0
                cntRight = 0
            
            stdTimeE = time.time()
        if ret:
            cv2.imshow('camera', img)   # 다음 프레임 이미지 표시
            #cv2.imshow('gray1', gray1)
            #cv2.imshow('gray2', gray2)
            #cv2.imshow('thresh_cv', thresh_cv)
            #cv2.imshow('t_otsu', thresh)
            cv2.imshow('th3', line_mask)
            #cv2.imshow('dst1', dst1)
            #cv2.imshow('dst2', dst2)
            
            
            if cv2.waitKey(1) != -1:    # 1ms 동안 키 입력 대기 ---②
                break                   # 아무 키라도 입력이 있으면 중지
        else:
            print('no frame')
            break
else:
    print("can't open camera.")
cap.release()                           # 자원 반납
cv2.destroyAllWindows()
