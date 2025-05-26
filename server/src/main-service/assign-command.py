import random
import time
from collections import deque

from db_utils import (
    get_vehicle_id_by_plate,
    get_latest_dock_from_camera_events,
    get_reservations,
    get_available_robots,
    get_empty_one_slot,
    get_pallet_slot_layer_by_barcode,
    is_robot_available,
    update_robot_status,
)
from robot_sender import dispatch_task

# -------------------------------
# 🔁 작업 큐 선언 (FIFO)
# -------------------------------
task_queue = deque()


# -------------------------------
# 📦 작업 생성 및 큐에 저장
# -------------------------------
def assign_and_enqueue_tasks(packet):
    plate_number = packet.get("plate_number")
    if not plate_number:
        print("[ERROR] No plate_number in packet")
        return

    dock = get_latest_dock_from_camera_events(plate_number)
    vehicle_id = get_vehicle_id_by_plate(plate_number)
    print(f"[DEBUG] vehicle_id for {plate_number}: {vehicle_id}")

    reservations = get_reservations(vehicle_id)
    print(f"[DEBUG] reservations: {reservations}")

    robots = get_available_robots()
    print(f"[DEBUG] available robots: {robots}")

    if not reservations or not robots:
        print("[WARN] No reservations or available robots")
        return

    robot_idx = 0
    for res in reservations:
        task_type = res['operation_type']
        barcode = res['barcode']
        quantity = res['quantity']

        for i in range(quantity):
            robot_id = robots[robot_idx % len(robots)]
            robot_idx += 1

            src, dst = generate_src_dst(task_type, barcode, dock)

            if not src or not dst:
                print(f"[WARN] Skipping task due to invalid src/dst → src: {src}, dst: {dst}")
                continue

            task_packet = {
                "robot_id": robot_id,
                "task_type": task_type,
                "barcode": barcode,
                "src": src,
                "dst": dst
            }

            task_queue.append(task_packet)
            print(f"[QUEUE] Task queued for {robot_id}: {task_packet}")


# -------------------------------
# 🤖 로봇 상태 감시 루프 (0.5초마다 idle 확인)
# -------------------------------
def monitor_robot_status_and_dispatch():
    while True:
        idle_robots = set(get_available_robots())
        print(f"[DEBUG] Currently idle robots: {idle_robots}")

        for robot_id in idle_robots:
            matched = False
            for task in list(task_queue):
                print(f"[DEBUG] Checking task: {task['robot_id']} vs {robot_id}")
                if task['robot_id'] == robot_id:
                    print(f"[DEBUG] Task matched for {robot_id}, dispatching")
                    dispatch_task(task)
                    update_robot_status(robot_id, "working")
                    task_queue.remove(task)
                    print(f"[DISPATCH] Task sent to {robot_id} (triggered by idle)")
                    matched = True
                    break
            if not matched:
                print(f"[DEBUG] No matching task found for {robot_id}")

        time.sleep(0.5)


# -------------------------------
# 🚚 출발지/도착지 생성 (barcode 기반)
# -------------------------------
def generate_src_dst(task_type, barcode, dock):
    if task_type == 'inbound':
        src = f'dock{dock}'
        dst = get_empty_one_slot(barcode)
    else:  # outbound
        src = get_pallet_slot_layer_by_barcode(barcode)
        dst = f'dock{dock}'
    return src, dst
