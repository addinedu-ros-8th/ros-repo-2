import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2
import mysql.connector
import re
from tcp_sender import send_reservation_data


from_class = uic.loadUiType("order_inout_2.ui")[0]

class Orderinout2Window(QMainWindow, from_class):
    def __init__(self, vehicle_id = None, company_name="", plate_number=""):
        super().__init__()
        self.setupUi(self)
        self.vehicle_id = vehicle_id
        self.company_name = company_name
        self.plate_number = plate_number

        self.in_btn.clicked.connect(self.inn)
        self.out_btn.clicked.connect(self.outt)

 
############################################################################
        self.pixmap = QPixmap()
    
        image = cv2.imread('./data/refri4.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        h,w,c = image.shape
        qimage = QImage(image.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap = self.pixmap.fromImage(qimage)
        self.pixmap = self.pixmap.scaled(self.a_pic.width(), self.a_pic.height())

        self.a_pic.setPixmap(self.pixmap)

        self.pixmap2 = QPixmap()
    
        image2 = cv2.imread('./data/washingmachine_4.png')
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)

        h,w,c = image2.shape
        qimage2 = QImage(image2.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap2 = self.pixmap2.fromImage(qimage2)
        self.pixmap2 = self.pixmap2.scaled(self.b_pic.width(), self.b_pic.height())

        self.b_pic.setPixmap(self.pixmap2)

        self.pixmap3 = QPixmap()
    
        image3 = cv2.imread('./data/aircon_6.png')
        image3 = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)

        h,w,c = image3.shape
        qimage3 = QImage(image3.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap3 = self.pixmap3.fromImage(qimage3)
        self.pixmap3 = self.pixmap3.scaled(self.c_pic.width(), self.c_pic.height())

        self.c_pic.setPixmap(self.pixmap3)

############################################################################
        for time in range(0,24):
            self.cbTime_in.addItem(str(time))
            self.cbTime_out.addItem(str(time))

        for minute in range(0,60,10):
            self.cbMin_in.addItem(str(minute))
            self.cbMin_out.addItem(str(minute))
        
        for year in range(2025,2027):
            self.cbYear_in.addItem(str(year))
            self.cbYear_out.addItem(str(year))

        for month in range(1,13):
            self.cbMonth_in.addItem(str(month))
            self.cbMonth_out.addItem(str(month))

        for date in range(1,31):
            self.cbDate_in.addItem(str(date))
            self.cbDate_out.addItem(str(date))


        self.cbTime_in.setCurrentText(str(0))
        self.cbTime_out.setCurrentText(str(0))
        self.cbMin_in.setCurrentText(str(0))
        self.cbMin_out.setCurrentText(str(0))

        self.cbTime_in.currentIndexChanged.connect(self.time_in_change)
        self.cbTime_out.currentIndexChanged.connect(self.time_out_change)
        self.cbMin_in.currentIndexChanged.connect(self.time_in_change)
        self.cbMin_out.currentIndexChanged.connect(self.time_out_change)
        self.cbYear_in.currentIndexChanged.connect(self.time_in_change)
        self.cbYear_out.currentIndexChanged.connect(self.time_out_change)
        self.cbMonth_in.currentIndexChanged.connect(self.time_in_change)
        self.cbMonth_out.currentIndexChanged.connect(self.time_out_change)
        self.cbDate_in.currentIndexChanged.connect(self.time_in_change)
        self.cbDate_out.currentIndexChanged.connect(self.time_out_change)




    def time_in_change(self):
        year_in = self.cbYear_in.currentText().zfill(2)
        month_in = self.cbMonth_in.currentText().zfill(2)
        date_in = self.cbDate_in.currentText().zfill(2)
        time_in = self.cbTime_in.currentText().zfill(2)  
        minute_in = self.cbMin_in.currentText().zfill(2)  

        self.lineEdit.setText(year_in+ '.' +month_in.zfill(2) + '.' + date_in.zfill(2) + '  ' + time_in.zfill(2)+ ":" + minute_in)
    
    def time_out_change(self):
        year_out = self.cbYear_out.currentText().zfill(2)
        month_out = self.cbMonth_out.currentText().zfill(2)
        date_out = self.cbDate_out.currentText().zfill(2)
        time_out = self.cbTime_out.currentText().zfill(2)  
        minute_out = self.cbMin_out.currentText().zfill(2)  

        self.lineEdit_2.setText(year_out+'.' + month_out.zfill(2)+'.'+date_out.zfill(2) + '  ' + time_out.zfill(2)+ ":" + minute_out)
    
############################################################################
        self.load_barcode_counts()

    def load_barcode_counts(self):
        try:
            remote = mysql.connector.connect(
                host="192.168.0.49",
                port=3306,
                user="sugoi_user",
                password="1",
                database='SUGOIDB'
            )
            cur = remote.cursor()

            barcodes_to_count = ['1234', '1235', '1236']
            counts = {}

            for barcode in barcodes_to_count:
                query = "SELECT COUNT(*) FROM pallets WHERE barcode = %s;"
                cur.execute(query, (barcode,))
                result = cur.fetchone()
                if result:
                    counts[barcode] = result[0]
                else:
                    counts[barcode] = 0

            if hasattr(self, 'a_sum') and isinstance(self.a_sum, QLineEdit):
                self.a_sum.setText(str(counts.get('1234', 0)))
            else:
                print("Error: 'sum'라는 QLineEdit 위젯을 찾을 수 없습니다.")

            if hasattr(self, 'b_sum') and isinstance(self.b_sum, QLineEdit):
                self.b_sum.setText(str(counts.get('1235', 0)))
            else:
                print("Error: 'b_sum'라는 QLineEdit 위젯을 찾을 수 없습니다.")

            if hasattr(self, 'c_sum') and isinstance(self.c_sum, QLineEdit):
                self.c_sum.setText(str(counts.get('1236', 0)))
            else:
                print("Error: 'c_sum'라는 QLineEdit 위젯을 찾을 수 없습니다.")

            cur.close()
            remote.close()

        except mysql.connector.Error as err:
            print(f"Error: {err}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")


############################################################################

    def inn(self):
        self.inn_data()
        QMessageBox.information(self, "입고 예약", "입고 예약이 완료되었습니다.")
        self.close()

    def outt(self):
        self.outt_data()
        QMessageBox.information(self, "출고 예약", "출고 예약이 완료되었습니다.")
        self.close()
    
    def inn_data(self):
        a_barcode = self.a_num.text()
        b_barcode = self.b_num.text()
        c_barcode = self.c_num.text()

        in_datetime_str = self.lineEdit.text()
        print(f"lineEdit 전체 텍스트: {in_datetime_str}")

        parts = re.split(r'\s+', in_datetime_str) # 하나 이상의 공백 문자를 기준으로 분리
        in_date_part = parts[0].replace('.', '-')  # '2025-01-01'
        in_time_part_hm = parts[1] 
        in_time_str = in_time_part_hm + ":00"
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
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1234",a_barcode, in_date_part, in_time_str,'inbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            print(f"첫번째 데이터 입력 완료")

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1234",
                "quantity": a_barcode,
                "date": in_date_part,
                "time": in_time_str,
                "operation_type": "inbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)

            query = """
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1235",b_barcode, in_date_part, in_time_str,'inbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            print(f"두번째 데이터 입력 완료")

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1235",
                "quantity": b_barcode,
                "date": in_date_part,
                "time": in_time_str,
                "operation_type": "inbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)

            query = """
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1236",c_barcode, in_date_part, in_time_str,'inbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            print(f"세번째 데이터 입력 완료")

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1236",
                "quantity": c_barcode,
                "date": in_date_part,
                "time": in_time_str,
                "operation_type": "inbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)

            cur.close()
            remote.close()

        except mysql.connector.Error as err:
            print(f"Error: {err}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    def outt_data(self):
        a_barcode = self.a_num.text()
        b_barcode = self.b_num.text()
        c_barcode = self.c_num.text()

        out_datetime_str = self.lineEdit_2.text()
        print(f"lineEdit 전체 텍스트: {out_datetime_str}")

        parts = re.split(r'\s+', out_datetime_str) # 하나 이상의 공백 문자를 기준으로 분리
        out_date_part = parts[0].replace('.', '-')  # '2025-01-01'
        out_time_part_hm = parts[1] 
        out_time_str = out_time_part_hm + ":00"
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
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1234",a_barcode, out_date_part, out_time_str,'outbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1234",
                "quantity": a_barcode,
                "date": out_date_part,
                "time": out_time_str,
                "operation_type": "outbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)

            print(f"첫번째 데이터 입력 완료")

            query = """
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1235",b_barcode, out_date_part, out_time_str,'outbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1235",
                "quantity": b_barcode,
                "date": out_date_part,
                "time": out_time_str,
                "operation_type": "outbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)

            print(f"두번째 데이터 입력 완료")

            query = """
                INSERT INTO reservations (barcode, quantity, date, time, operation_type, vehicle_id, status)
                VALUES (%s, %s, %s, %s, %s, %s, %s);
            """
            values = ("1236",c_barcode, out_date_part, out_time_str,'outbound',self.vehicle_id,'completed')
            cur.execute(query, values)
            remote.commit()

            packet = {
                "event": "reservation",
                "company_name": self.company_name,
                "plate_number": self.plate_number,
                "barcode": "1236",
                "quantity": c_barcode,
                "date": out_date_part,
                "time": out_time_str,
                "operation_type": "outbound",
                "vehicle_id": self.vehicle_id,
                "status": "completed"
            }

            send_reservation_data(packet)
            
            print(f"세번째 데이터 입력 완료")

            cur.close()
            remote.close()

        except mysql.connector.Error as err:
            print(f"Error: {err}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

############################################################################

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = Orderinout2Window()
    myWindows.show()

    sys.exit(app.exec())