import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2
import mysql.connector

from_class = uic.loadUiType("client.ui")[0]

class clientWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pixmap = QPixmap()

        self.pushButton.clicked.connect(self.push_btn)
    
        image = cv2.imread('./data/login_3.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        h,w,c = image.shape
        qimage = QImage(image.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap = self.pixmap.fromImage(qimage)
        self.pixmap = self.pixmap.scaled(self.label.width(), self.label.height())

        self.label.setPixmap(self.pixmap)

    def push_btn(self):
        plate_number = self.textEdit.toPlainText().strip()

        if not plate_number:
            QMessageBox.warning(self, "입력 오류", "차량 번호를 입력하세요.")
            return
    
        if self.check_vehicle_exists(plate_number):
            from client_check import checkWindow
            print("client check page connect")
            self.close()
            self.main_window = checkWindow(plate_number)
            self.main_window.show()
        else:
            QMessageBox.information(self,"입력 오류" ,"차량 번호를 다시 입력하세요.")

    def check_vehicle_exists(self, plate_number):
        try:
            conn = mysql.connector.connect(
                host="192.168.0.49",
                port=3306,
                user="sugoi_user",
                password="1",
                database="SUGOIDB"
            )
            cursor = conn.cursor()
            cursor.execute("SELECT id FROM vehicles WHERE plate_number = %s", (plate_number,))
            result = cursor.fetchone()
            cursor.close()
            conn.close()
            return result is not None
        except mysql.connector.Error as err:
            QMessageBox.critical(self, "DB 오류", f"데이터베이스 연결 실패: {err}")
            return False

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = clientWindow()
    myWindows.show()

    sys.exit(app.exec())