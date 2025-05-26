import cv2
import time
import easyocr
import socket
import numpy as np
import re
import threading
import queue
from tcp_sender import send_result
from db_logger import insert_plate_event
from difflib import SequenceMatcher

# EasyOCR 초기화
reader = easyocr.Reader(['ko', 'en'], gpu=True)

# UDP 설정
UDP_IP = "0.0.0.0"
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ROI 영역 설정
roi_config = {
    1: {"label": "DOCK 2", "coords": (100, 240, 280, 340), "color": (0, 255, 0)},
    2: {"label": "DOCK 1", "coords": (370, 250, 550, 350), "color": (255, 0, 0)},
}

# 최근 감지 기록
COOLDOWN_SECONDS = 10
recent_detections = {
    1: {"plate": None, "timestamp": 0},
    2: {"plate": None, "timestamp": 0}
}

# ROI 큐 생성
roi_queue = queue.Queue()

def receive_frame():
    try:
        data, _ = sock.recvfrom(65536)
        np_data = np.frombuffer(data, dtype=np.uint8)
        return cv2.imdecode(np_data, cv2.IMREAD_COLOR)
    except Exception as e:
        print("[UDP ERROR]", e)
        return None

def normalize_plate(text):
    return re.sub(r"[^0-9가-힣]", "", text).strip()

def is_similar(a, b, threshold=0.8):
    return SequenceMatcher(None, a, b).ratio() >= threshold

def detect_plate(roi):
    results = reader.readtext(roi)
    for _, text, confidence in results:
        if confidence < 0.7:
            continue
        norm_text = normalize_plate(text)
        if any(c.isdigit() for c in norm_text) and any('\uAC00' <= c <= '\uD7A3' for c in norm_text):
            return {"text": norm_text, "confidence": round(confidence, 2)}
    return None

def ocr_worker():
    while True:
        idx, roi = roi_queue.get()
        if roi is None:
            break

        result = detect_plate(roi)
        if result:
            plate_number = result["text"]
            confidence = result["confidence"]
            now = time.time()

            recent = recent_detections[idx]
            if recent["plate"] and is_similar(plate_number, recent["plate"]) and (now - recent["timestamp"] < COOLDOWN_SECONDS):
                roi_queue.task_done()
                continue

            recent_detections[idx] = {"plate": plate_number, "timestamp": now}
            packet = {
                "event": "plate_detected",
                "dock": idx,
                "plate_number": plate_number,
                "confidence": confidence,
                "timestamp": now
            }

            print(f"[INFO] {roi_config[idx]['label']} 감지됨:", packet)
            send_result(packet)
            insert_plate_event(
                dock=packet["dock"],
                plate_number=packet["plate_number"],
                confidence=float(packet["confidence"])
            )
        roi_queue.task_done()

def main():
    print("[RUN] 멀티스레드 번호판 인식 시작")

    # OCR 스레드 시작
    ocr_thread = threading.Thread(target=ocr_worker, daemon=True)
    ocr_thread.start()

    while True:
        frame = receive_frame()
        if frame is None:
            continue

        for idx, config in roi_config.items():
            x1, y1, x2, y2 = config["coords"]
            roi = frame[y1:y2, x1:x2]

            # OCR 작업 큐에 넣기
            roi_queue.put((idx, roi))

            # ROI 사각형 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), config["color"], 2)
            cv2.putText(frame, config["label"], (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, config["color"], 2)

        cv2.imshow("Frame with ROIs", frame)
        if cv2.waitKey(10) == 27:
            break

    # 종료
    roi_queue.put((None, None))  # 스레드 종료 신호
    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
