import subprocess
import time
import signal
import sys

processes = []

def launch(cmd_list, name):
    print(f"[LAUNCH] {name}")
    return subprocess.Popen(cmd_list)

def main():
    try:
        # === Marker Publisher는 먼저 수동으로 실행하세요 ===

        # === Python UDP 영상 송신자 ===
        # processes.append(launch(["python3", "camera_udp_sender_marker.py"], "📤 Marker Sender"))
        processes.append(launch(["python3", "camera_udp_sender_plate.py"], "📤 Plate Sender"))

        # === 번호판 인식기 ===
        processes.append(launch(["python3", "run_plate5.py"], "🔍 Plate Detector"))

        print("\n[INFO] 모든 AI 서버 측 프로세스를 실행했습니다. Ctrl+C 로 종료하세요.\n")

        # 메인 루프는 단순히 대기만
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C 감지됨. 모든 프로세스 종료 중...")
        for proc in processes:
            proc.send_signal(signal.SIGINT)
        for proc in processes:
            proc.wait()
        print("[DONE] 종료 완료")
        sys.exit(0)

if __name__ == "__main__":
    main()
