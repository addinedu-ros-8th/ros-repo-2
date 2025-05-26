import mysql.connector
from datetime import datetime

# DB 연결 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': '1',  # 실제 비밀번호로 수정
    'database': 'SUGOIDB',
    'port': 3306
}

def insert_plate_event(dock, plate_number, confidence):
    try:
        conn = mysql.connector.connect(**DB_CONFIG)
        cursor = conn.cursor()

        query = """
            INSERT INTO camera_events (event_type, dock, plate_number, confidence, timestamp)
            VALUES (%s, %s, %s, %s, %s)
        """
        data = ("plate_detected", dock, plate_number, confidence, datetime.now())
        cursor.execute(query, data)

        conn.commit()
        cursor.close()
        conn.close()

        print(f"[DB] 저장 완료: 도크 {dock}, 번호판 {plate_number}, 신뢰도 {confidence}")

    except mysql.connector.Error as err:
        print(f"[DB ERROR] {err}")
