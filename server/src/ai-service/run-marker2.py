import cv2
import numpy as np
import socket

# --- UDP 수신 설정 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 8887
BUFF_SIZE = 65536

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print("[RECEIVER] UDP 수신 대기 중...")

# --- 카메라 보정 데이터 불러오기 ---
with np.load("camera_calibration_data.npz") as data:
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']

# --- ArUco 설정 ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# 마커 한 변의 실제 길이 (미터 단위)
marker_length = 0.08

# 마커 좌표계 기준 3D 포인트 정의 (정사각형 기준 시계방향)
marker_3d = np.array([
    [-marker_length / 2, marker_length / 2, 0],
    [ marker_length / 2, marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

while True:
    try:
        data, _ = sock.recvfrom(BUFF_SIZE)
        np_data = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        if frame is None:
            continue

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, corner in enumerate(corners):
                img_points = corner[0].astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(marker_3d, img_points, camera_matrix, dist_coeffs)

                if success:
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    x, y, z = tvec.flatten()
                    print(f"[MARKER] ID: {ids[i][0]} → X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, ROT: {rvec.flatten()}")

        cv2.imshow("Marker Detection", frame)
        if cv2.waitKey(1) == 27:
            break

    except Exception as e:
        print("[ERROR]", e)
        continue

sock.close()
cv2.destroyAllWindows()
