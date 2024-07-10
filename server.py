import socket

def start_socket_server():
    host = '172.30.1.5'  # 소켓 서버의 IP 주소
    port = 5001          # 소켓 서버의 포트 번호
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("TCP/IP 소켓 서버가 시작되었습니다.")

    while True:
        client_socket, addr = server_socket.accept()
        print(f'Connected by {addr}')

        try:
            while True:
                data = client_socket.recv(1024).decode()  # 클라이언트로부터 데이터 수신
                if not data:
                    break
                print(f"Received command: {data}")

                # 여기에서 data를 기반으로 필요한 작업을 수행합니다.
                # 예: 아두이노에 데이터를 전송하거나, 다른 시스템과 통신 등
                response = "Command received: " + data
                client_socket.sendall(response.encode())  # 응답 전송

        finally:
            client_socket.close()

    server_socket.close()

if __name__ == '__main__':
    start_socket_server()
