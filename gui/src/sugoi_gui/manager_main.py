import sys
import threading
import socket
import struct
import pickle
import numpy as np
import cv2

from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6 import uic

# --- Pose 관련 상수 ---
ORIGIN = [3.1, 1.9]
RESOLUTION = 0.005  # meters per pixel

# --- 로봇 IP ---
BOT1_IP = "192.168.0.69"
BOT2_IP = "192.168.0.68"

# --- 포트 정보 ---
BOT1_POSE_PORT = 10002
BOT2_POSE_PORT = 10001


def world_to_map(pose_x, pose_y, origin, resolution, map_width, map_height):
    dx = -(pose_x - origin[0])
    dy = -(pose_y - origin[1])
    map_x = int(dx / resolution)
    map_y = int(map_height - (dy / resolution))
    map_x = max(0, min(map_width - 1, map_x))
    map_y = max(0, min(map_height - 1, map_y))
    return map_x, map_y


class PoseReceiverThread(threading.Thread):
    def __init__(self, update_pose_callback, ip, port):
        super().__init__()
        self.update_pose_callback = update_pose_callback
        self.SERVER_IP = ip
        self.POSE_PORT = port
        self.daemon = True
        self.running = True

    def run(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client_socket.connect((self.SERVER_IP, self.POSE_PORT))
            print(f"[PoseReceiver-{self.POSE_PORT}] Connected to pose server")
        except Exception as e:
            print(f"[PoseReceiver-{self.POSE_PORT}] Connection error: {e}")
            return

        data = b""
        payload_size = struct.calcsize("Q")

        while self.running:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    self.running = False
                    break
                data += packet

            if not self.running:
                break

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4096)

            pose_data_bytes = data[:msg_size]
            data = data[msg_size:]

            pose = pickle.loads(pose_data_bytes)
            if pose:
                x = pose.get('position_x', 0.0)
                y = pose.get('position_y', 0.0)
                self.update_pose_callback(x, y)

        client_socket.close()

    def stop(self):
        self.running = False


class ClickableLabel(QLabel):
    clicked = pyqtSignal()
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()


from_class = uic.loadUiType("manager_main.ui")[0]

class ManagerWindow(QMainWindow, from_class):
    pose_updated = pyqtSignal(float, float)
    pose2_updated = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.normal_btn.clicked.connect(self.normal)
        self.setup_bot_images()
        self.setup_map_image()

        self.sideMenu.setGeometry(-150, 0, 150, self.height())
        self.sideMenu.hide()
        self.menu_anim = QPropertyAnimation(self.sideMenu, b"geometry")
        self.menu_anim.setDuration(300)

        self.pose_x = self.pose_y = self.pose2_x = self.pose2_y = 0.0
        self.pose_updated.connect(self.update_pose)
        self.pose2_updated.connect(self.update_pose2)

        self.timer = QTimer()
        self.timer.timeout.connect(self.draw_pose_on_map)
        self.timer.start(200)

        self.pose_receiver_thread = PoseReceiverThread(self.emit_pose_updated, BOT1_IP, BOT1_POSE_PORT)
        self.pose_receiver_thread2 = PoseReceiverThread(self.emit_pose2_updated, BOT2_IP, BOT2_POSE_PORT)
        self.pose_receiver_thread.start()
        self.pose_receiver_thread2.start()

    def setup_bot_images(self):
        image1 = cv2.imread('./data/map_car_1.png')
        image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
        pixmap1 = QPixmap.fromImage(QImage(image1.data, image1.shape[1], image1.shape[0], image1.shape[1]*3, QImage.Format.Format_RGB888)).scaled(self.bot1.width(), self.bot1.height())
        self.bot1_clickable = ClickableLabel()
        self.bot1_clickable.setPixmap(pixmap1)
        QVBoxLayout(self.bot1).addWidget(self.bot1_clickable)
        self.bot1_clickable.clicked.connect(lambda: print("Bot1 clicked"))

        image2 = cv2.imread('./data/map_car_2.png')
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
        pixmap2 = QPixmap.fromImage(QImage(image2.data, image2.shape[1], image2.shape[0], image2.shape[1]*3, QImage.Format.Format_RGB888)).scaled(self.bot2.width(), self.bot2.height())
        self.bot2_clickable = ClickableLabel()
        self.bot2_clickable.setPixmap(pixmap2)
        QVBoxLayout(self.bot2).addWidget(self.bot2_clickable)
        self.bot2_clickable.clicked.connect(lambda: print("Bot2 clicked"))

    def setup_map_image(self):
        image3 = cv2.imread('./data/realmap_3.png')
        image3 = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)
        self.pixmap3 = QPixmap.fromImage(QImage(image3.data, image3.shape[1], image3.shape[0], image3.shape[1]*3, QImage.Format.Format_RGB888))
        self.map_width = self.pixmap3.width()
        self.map_height = self.pixmap3.height()
        self.rviz.setPixmap(self.pixmap3)
        self.rviz.setAlignment(Qt.AlignmentFlag.AlignCenter)

    def emit_pose_updated(self, x, y):
        self.pose_updated.emit(x, y)

    def emit_pose2_updated(self, x, y):
        self.pose2_updated.emit(x, y)

    def update_pose(self, x, y):
        print(f"[BOT1] Pose received: ({x}, {y})")
        self.pose_x = x
        self.pose_y = y

    def update_pose2(self, x, y):
        print(f"[BOT2] Pose received: ({x}, {y})")
        self.pose2_x = x
        self.pose2_y = y

    def draw_pose_on_map(self):
        pixmap_copy = self.pixmap3.copy()
        painter = QPainter(pixmap_copy)

        pen = QPen(Qt.GlobalColor.red)
        pen.setWidth(8)
        painter.setPen(pen)
        painter.setBrush(QBrush(Qt.GlobalColor.red))
        x1, y1 = world_to_map(self.pose_x, self.pose_y, ORIGIN, RESOLUTION, self.map_width, self.map_height)
        painter.drawEllipse(QPoint(x1, y1), 10, 10)

        pen.setColor(Qt.GlobalColor.blue)
        painter.setPen(pen)
        painter.setBrush(QBrush(Qt.GlobalColor.blue))
        x2, y2 = world_to_map(self.pose2_x , self.pose2_y + 0.9, ORIGIN, RESOLUTION, self.map_width, self.map_height)
        painter.drawEllipse(QPoint(x2, y2), 10, 10)
        print(f"[Map] BOT1 mapped: ({x1}, {y1}), BOT2 mapped: ({x2}, {y2})")
        painter.end()
        self.rviz.setPixmap(pixmap_copy)

    def mouseMoveEvent(self, event):
        x = event.position().x()
        if x < 20:
            self.showMenu()
        elif x > 170:
            self.hideMenu()

    def showMenu(self):
        if self.sideMenu.isHidden():
            self.sideMenu.show()
        self.menu_anim.stop()
        self.menu_anim.setStartValue(self.sideMenu.geometry())
        self.menu_anim.setEndValue(QRect(0, 0, 150, self.height()))
        self.menu_anim.start()

    def hideMenu(self):
        self.menu_anim.stop()
        self.menu_anim.setStartValue(self.sideMenu.geometry())
        self.menu_anim.setEndValue(QRect(-150, 0, 150, self.height()))
        self.menu_anim.start()

    def normal(self):
        print("HI")

    def closeEvent(self, event):
        self.pose_receiver_thread.stop()
        self.pose_receiver_thread2.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ManagerWindow()
    window.show()
    sys.exit(app.exec())
