import socket
import select
import sys
import serial
import time


# py_serial = serial.Serial(
#     # Window
#     port='COM10',
#     # 보드 레이트 (통신 속도)
#     baudrate=115200,
# )
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('172.30.1.5', 5001))
# msg = 'connected??'
# s.send(msg.encode())
while True:
      
    commend = input('아두이노에게 내릴 명령:')
    s.send(commend.encode())
    time.sleep(0.5)
    data = s.recv(4096)
    # # data = data[:len(data)-1]
    print(data)
    