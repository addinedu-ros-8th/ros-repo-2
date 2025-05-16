import qrcode
import json

# 저장할 정보 (예: 팔레트 정보)
data = {
    "pallet_id": "pallet_1",
}

# JSON 문자열로 인코딩
qr_data = json.dumps(data)

# QR 코드 생성
qr = qrcode.QRCode(
    version=1,  # 버전 1: 가장 작은 크기
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(qr_data)
qr.make(fit=True)

# 이미지로 저장
img = qr.make_image(fill="black", back_color="white")
img.save("pallet_1_qr.png")
