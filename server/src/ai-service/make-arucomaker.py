import cv2
import cv2.aruco as aruco

# 사용할 딕셔너리 선택
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_7X7_100)

# ID 범위 설정 및 이미지 생성
for marker_id in range(2):  # 0~4번 마커 생성
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, 300)  # 300x300 px
    cv2.imwrite(f"marker_{marker_id:02d}.png", marker_img)
    print(f"[SAVED] marker_{marker_id:02d}.png")
