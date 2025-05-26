import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import cv2

from_class = uic.loadUiType("client_picture.ui")[0]

class clientpicutreWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pixmap = QPixmap()

        self.pushButton.clicked.connect(self.push_btn)
        self.nextButton.clicked.connect(self.next_btn)
    
        image = cv2.imread('./data/work_cap.jpg')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        h,w,c = image.shape
        qimage = QImage(image.data, w, h, w*c, QImage.Format.Format_RGB888)

        self.pixmap = self.pixmap.fromImage(qimage)
        self.pixmap = self.pixmap.scaled(self.label.width(), self.label.height())

        self.label.setPixmap(self.pixmap)
    def push_btn(self):
        self.close()
    
    def next_btn(self):
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = clientpicutreWindow()
    myWindows.show()

    sys.exit(app.exec())