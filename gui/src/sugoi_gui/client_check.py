import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2
import mysql.connector
from tcp_sender import send_packet  
import datetime
from PyQt6.QtCore import QTimer
from PyQt6.QtCore import Qt


from_class = uic.loadUiType("client_check.ui")[0]

class checkWindow(QMainWindow, from_class):
    def __init__(self, plate_number):
        super().__init__()
        self.setupUi(self)
        self.label_4.hide()
        self.label_5.hide()
        self.plate_number = plate_number
        self.pixmap = QPixmap()

        self.pushButton.clicked.connect(self.push_btn)

        # think22
        image = cv2.imread('./data/think22.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w, c = image.shape
        qimage = QImage(image.data, w, h, w*c, QImage.Format.Format_RGB888)
        self.pixmap = QPixmap.fromImage(qimage).scaled(self.label.width(), self.label.height())
        self.label.setPixmap(self.pixmap)

        # ball7
        image2 = cv2.imread('./data/ball7.png')
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
        h, w, c = image2.shape
        qimage2 = QImage(image2.data, w, h, w*c, QImage.Format.Format_RGB888)
        self.pixmap2 = QPixmap.fromImage(qimage2).scaled(self.label_2.width(), self.label_2.height())
        self.label_2.setPixmap(self.pixmap2)

        # font2
        image3 = cv2.imread('./data/font2.png')
        image3 = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)
        h, w, c = image3.shape
        qimage3 = QImage(image3.data, w, h, w*c, QImage.Format.Format_RGB888)
        self.pixmap3 = QPixmap.fromImage(qimage3).scaled(self.label_3.width(), self.label_3.height())
        self.label_3.setPixmap(self.pixmap3)

        # 애니메이션 관련
        self.dots = []
        self.animation_index = 0
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.animate_dots)
        self.forklift_label = None

        self.load_reservations()

    def push_btn(self):
        self.label.hide()
        self.label_2.hide()
        self.label_3.hide()
        self.label_4.show()
        self.label_5.show()


        self.pushButton.hide()

        window_width = self.width()
        window_height = self.height()

        dot_spacing = 150  # 점 간 간격
        dot_size = 100
        forklift_size = 100
        total_width = (dot_spacing * (5 - 1)) + dot_size

        start_x = (window_width - total_width) // 2
        y_position = (window_height // 2)  # 가운데 높이

        # 점 생성
        if not self.dots:
            for i in range(5):
                dot = QLabel("●", self)
                dot.setStyleSheet(f"font-size: {dot_size}px; color: gray;")
                dot.setGeometry(start_x + i * dot_spacing, y_position, dot_size, dot_size)
                dot.show()
                self.dots.append(dot)

        self.animation_index = 0
        self.animation_timer.start(300)

        now = datetime.datetime.now()
        packet = {
            "event": "work_request_start",
            "plate_number": self.plate_number,
            "date": now.strftime("%Y-%m-%d"),
            "time": now.strftime("%H:%M")
        }
        send_packet(packet)


    def animate_dots(self):
        forklift_pixmap = QPixmap("./data/map_car_1.png").scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio)

        for i, dot in enumerate(self.dots):
            if i == self.animation_index:
                dot.setPixmap(forklift_pixmap)
                dot.setText("")  # 텍스트 제거
            else:
                dot.clear()  # 이미지 제거
                dot.setText("●")
                dot.setStyleSheet("font-size: 70px; color: gray;")

        self.animation_index = (self.animation_index + 1) % 5

    def load_reservations(self):
        try:
            conn = mysql.connector.connect(
                host="192.168.0.49",
                port=3306,
                user="sugoi_user",
                password="1",
                database='SUGOIDB'
            )
            cursor = conn.cursor()
            query = """
                SELECT r.barcode, r.quantity, r.date, r.time, r.created_at
                FROM reservations r
                JOIN vehicles v ON r.vehicle_id = v.id
                WHERE v.plate_number = %s
            """
            cursor.execute(query, (self.plate_number,))
            rows = cursor.fetchall()

            self.tableWidget.setRowCount(len(rows))
            if rows:
                self.tableWidget.setColumnCount(len(rows[0]))
                column_names = ["바코드", "수량", "예약 날짜", "예약 시간", "예약한 날짜"]
                self.tableWidget.setHorizontalHeaderLabels(column_names)

                for row_idx, row_data in enumerate(rows):
                    for col_idx, value in enumerate(row_data):
                        self.tableWidget.setItem(row_idx, col_idx, QTableWidgetItem(str(value)))
            else:
                self.tableWidget.setColumnCount(0)
                self.tableWidget.setRowCount(0)
                self.tableWidget.setHorizontalHeaderLabels([])
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

            cursor.close()
            conn.close()
        except mysql.connector.Error as err:
            QMessageBox.critical(self, "DB 오류", f"데이터를 불러올 수 없습니다: {err}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    plate = "1234"  # 테스트용 번호판
    myWindow = checkWindow(plate)
    myWindow.show()
    sys.exit(app.exec())
