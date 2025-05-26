import cv2
import numpy as np
import glob
import os

# === 체커보드 설정 ===
CHECKERBOARD = (8, 6)  # 체스보드 내부 코너 개수 (가로, 세로)
square_size = 27.5  # mm 단위 (정확히 측정한 체커보드 칸의 한 변 길이로 변경)

# === 3D 월드 좌표계에서의 점들 준비 ===
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D 점들
imgpoints = []  # 2D 점들

# === 이미지 경로 ===
images = glob.glob('calib_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# === 캘리브레이션 수행 ===
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n[RESULT] Camera Matrix:\n", camera_matrix)
print("\n[RESULT] Distortion Coefficients:\n", dist_coeffs)

# === 결과 저장 ===
np.savez('aicamera_calibration_data.npz', 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs)
print("\n[SAVED] Calibration data saved to camera_calibration_data.npz")
