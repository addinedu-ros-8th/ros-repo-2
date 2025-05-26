import mysql.connector
import random

# 🛠️ DB 접속 정보 설정
def get_connection():
    return mysql.connector.connect(
        host="localhost",
        user="root",
        password="1",
        database="SUGOIDB",
        auth_plugin="mysql_native_password"
        
    )


# 📌 차량 번호로 vehicle_id 찾기
def get_vehicle_id_by_plate(plate_number):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM vehicles WHERE plate_number = %s", (plate_number,))
    result = cursor.fetchone()
    conn.close()
    return result[0] if result else None


# 📌 plate_number 기반 최근 dock 조회
def get_latest_dock_from_camera_events(plate_number):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT dock FROM camera_events
        WHERE plate_number = %s
        ORDER BY timestamp DESC LIMIT 1
    """, (plate_number,))
    result = cursor.fetchone()
    conn.close()
    return result[0] if result else None


# 📌 vehicle_id로 예약 목록 가져오기
def get_reservations(vehicle_id):
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute("""
        SELECT barcode, quantity, operation_type
        FROM reservations
        WHERE vehicle_id = %s AND status = 'reserved'
    """, (vehicle_id,))
    results = cursor.fetchall()
    conn.close()
    return results


# 📌 사용 가능한 로봇 리스트
def get_available_robots():
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT name FROM robots WHERE status = 'idle'")
    results = cursor.fetchall()
    conn.close()
    return [r[0] for r in results]


# 📌 바코드에 해당하는 전용 슬롯에서 빈 공간 1개 선택
def get_empty_one_slot(barcode):
    SLOT_MAP = {
        '1234': ['P1-lower', 'P1-upper', 'P4-lower', 'P4-upper'],
        '1235': ['P2-lower', 'P2-upper', 'P5-lower', 'P5-upper'],
        '1236': ['P3-lower', 'P3-upper', 'P6-lower', 'P6-upper'],
    }

    candidate_slots = SLOT_MAP.get(str(barcode), [])
    if not candidate_slots:
        return None

    # 슬롯 분리
    slot_layer_pairs = [s.split("-") for s in candidate_slots]

    conn = get_connection()
    cursor = conn.cursor()
    placeholders = ",".join(["(%s, %s)"] * len(slot_layer_pairs))
    values = [val for pair in slot_layer_pairs for val in pair]

    query = f"""
        SELECT slot, layer FROM pallets
        WHERE (slot, layer) IN ({placeholders})
        AND barcode IS NULL AND status = 'stored' AND location = 'warehouse'
    """
    cursor.execute(query, values)
    results = cursor.fetchall()
    conn.close()

    if not results:
        return None

    selected = random.choice(results)
    return f"{selected[0]}-{selected[1]}"


# 📌 바코드의 현재 슬롯 위치 반환 (예: 'P1-lower')
def get_pallet_slot_layer_by_barcode(barcode):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT slot, layer FROM pallets
        WHERE barcode = %s AND status = 'stored' AND location = 'warehouse'
        LIMIT 1
    """, (barcode,))
    result = cursor.fetchone()
    conn.close()
    return f"{result[0]}-{result[1]}" if result else None


# 📌 로봇이 idle인지 확인
def is_robot_available(robot_id):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT status FROM robots WHERE name = %s", (robot_id,))
    result = cursor.fetchone()
    conn.close()
    return result and result[0] == 'idle'


# 📌 로봇 상태 업데이트
def update_robot_status(robot_id, new_status):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("UPDATE robots SET status = %s WHERE name = %s", (new_status, robot_id))
    conn.commit()
    conn.close()
