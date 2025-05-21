import cv2
import os

# 저장할 폴더 이름
save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

# USB 웹캠(0번 장치), 또는 PiCam이면 숫자 바꾸거나 VideoCapture 경로 수정
cap = cv2.VideoCapture(2)  # 예: '/dev/video0'

if not cap.isOpened():
    print("[ERROR] 카메라를 열 수 없습니다.")
    exit()

print("[INFO] 's' 키를 눌러 이미지 저장, 'q' 키를 눌러 종료")
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARNING] 프레임 수신 실패")
        break

    cv2.imshow("Calibration Capture", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = os.path.join(save_dir, f"calib_{count:02d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"[SAVED] {filename}")
        count += 1
    elif key == ord('q'):
        print("[INFO] 종료합니다.")
        break

cap.release()
cv2.destroyAllWindows()

#v4l2-ctl --list-devices

