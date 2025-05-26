import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2

from_class = uic.loadUiType("main.ui")[0]

class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.manger_btn.clicked.connect(self.manager)
        self.order_btn.clicked.connect(self.order)
        self.client_btn.clicked.connect(self.client)

    def manager(self):
        from Manager_Main import ManagerWindow
        print("client check page connect")
        self.close()
        self.main_window = ManagerWindow()
        self.main_window.show()

    def order(self):
        from Order_Main import OrderMainWindow
        print("client check page connect")
        self.close()
        self.main_window = OrderMainWindow()
        self.main_window.show()

    def client(self):
        from client import clientWindow
        print("client check page connect")
        self.close()
        self.main_window = clientWindow()
        self.main_window.show()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = MainWindow()
    myWindows.show()

    sys.exit(app.exec())