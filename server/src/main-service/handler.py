# from robot_sender import dispatch_task
import json
from assign_command import assign_and_enqueue_tasks

def handle_event(packet):
    event_type = packet.get("event")

    if event_type == "plate_detected":
        handle_plate_detected(packet)

    elif event_type == "reservation":
        handle_reservation(packet)

    elif event_type == "work_request_start":
        assign_and_enqueue_tasks(packet)	

    else:
        print(f"[WARN] Unknown event type: {event_type}")


# --- 아래는 각각의 이벤트 핸들러 구현 ---

def handle_plate_detected(packet):
    plate = packet.get("plate_number")
    dock = packet.get("dock")
    confidence = packet.get("confidence")
    timestamp = packet.get("timestamp")
    print(f"[EVENT] plate_detected: {plate} at dock {dock} (confidence={confidence})")


def handle_reservation(packet):
    plate = packet.get("plate_number")
    barcode = packet.get("barcode")
    quantity = packet.get("quantity")
    operation_type = packet.get("operation_type")
    print(f"[EVENT] reservation: plate={plate}, barcode={barcode}, qty={quantity}, type={operation_type}")