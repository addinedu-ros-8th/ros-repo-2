import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2
import mysql.connector

from_class = uic.loadUiType("order_main.ui")[0]

class ClearTextEdit(QTextEdit):
    def mousePressEvent(self, event):
        self.clear()
        super().mousePressEvent(event)
        
class OrderMainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pixmap = QPixmap()

        self.inout_btn.clicked.connect(self.inoutt)
    
        image = cv2.imread('./data/login_2.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        h,w,c = image.shape
        qimage = QImage(image.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap = self.pixmap.fromImage(qimage)
        self.pixmap = self.pixmap.scaled(self.label.width(), self.label.height())

        self.label.setPixmap(self.pixmap)
        
    def inoutt(self):
        self.pushCompanyData()

        from order_inout_2 import Orderinout2Window
        print("log page connect")
        self.close()
        self.main_window = Orderinout2Window(vehicle_id=self.vehicle_id,
        company_name=self.company.toPlainText(),
        plate_number=self.number.toPlainText())
        self.main_window.show()
  

    def pushCompanyData(self):
        company_text = self.company.toPlainText()
        number_text = self.number.toPlainText()
        try:
            remote = mysql.connector.connect(
                host="192.168.0.49",
                port=3306,
                user="sugoi_user",
                password="1",
                database='SUGOIDB'
            )
            cur = remote.cursor()
            query = """
                INSERT INTO vehicles (plate_number, company_name)
                VALUES (%s, %s);
            """
            values = (number_text, company_text)
            cur.execute(query, values)
            remote.commit()
            vehicle_id = cur.lastrowid

            print(f"차량 정보 추가 완료 - 번호: {number_text}, 회사: {company_text}")

            self.vehicle_id = vehicle_id

            cur.close()
            remote.close()
        except mysql.connector.Error as err:
            print(f"MySQL Error: {err}")
            QMessageBox.critical(self, "Database Error", f"데이터베이스 오류가 발생했습니다: {err}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = OrderMainWindow()
    myWindows.show()

    sys.exit(app.exec())