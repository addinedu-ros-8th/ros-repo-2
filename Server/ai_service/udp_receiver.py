import cv2
import numpy as np

def receive_from_lifecam():
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("LifeCam을 열 수 없습니다.")
        return

    # 카메라 기본 설정 (원하면 수정 가능)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print("LifeCam 영상 수신 시작...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        yield frame

    cap.release()

if __name__ == "__main__":
    for frame in receive_from_lifecam():
        # 테스트용으로 프레임 띄워보기
        cv2.imshow('LifeCam', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
