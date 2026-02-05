# test_signal.py
import socket

# config.py에 설정한 것과 동일한 정보
IP = "127.0.0.1"
PORT = 5000

def send_test_signal(message):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((IP, PORT))
        sock.sendall(message.encode('utf-8'))
        sock.close()
        print(f"✅ 신호 전송 성공: '{message}' -> {IP}:{PORT}")
    except Exception as e:
        print(f"❌ 전송 실패: {e}")

if __name__ == "__main__":
    print("1: arrived (도착 신호 - 센서 감지 시작)")
    print("2: finished (작업 완료 - 강제 오픈)")
    
    choice = input("보낼 신호를 선택하세요 (1 또는 2): ")
    
    if choice == '1':
        send_test_signal("arrived")
    elif choice == '2':
        send_test_signal("finished")
    else:
        print("잘못된 입력입니다.")