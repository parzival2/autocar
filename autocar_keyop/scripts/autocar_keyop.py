#! /usr/bin/env python
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from PyQt5.QtCore import QTimer, QObject, Qt, pyqtSignal
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QWidget, QApplication
from keyop_rosnode import RosThreaddedNode
import rospy

class AutocarKeyop(QWidget):
    keyUpPressed = pyqtSignal()
    keyLeftPressed = pyqtSignal()
    keyRightPressed = pyqtSignal()
    def __init__(self):
        super(AutocarKeyop, self).__init__()
        self.nodeThread = RosThreaddedNode()
        self.keyUpPressed.connect(self.nodeThread.keyUpPressed)
        self.keyLeftPressed.connect(self.nodeThread.keyLeftPressed)
        self.keyRightPressed.connect(self.nodeThread.keyRightPressed)
        # Timeout timer
        self.speedTimer = QTimer()
        self.speedTimer.setSingleShot(True)
        self.speedTimer.setInterval(1000)
        self.speedTimer.timeout.connect(self.nodeThread.resetSpeed)
        # Steer timeout timer
        self.steerTimer = QTimer()
        self.steerTimer.setSingleShot(True)
        self.steerTimer.setInterval(100)
        self.steerTimer.timeout.connect(self.nodeThread.resetSteer)
        self.nodeThread.start()

    def keyPressEvent(self, event):
        """
        Processes key events
        """
        if(type(event)) == QKeyEvent:
            if(event.key() == Qt.Key_Up):
                self.speedTimer.start()
                self.keyUpPressed.emit()
            if(event.key() == Qt.Key_Left):
                self.steerTimer.start()
                self.keyLeftPressed.emit()
            if(event.key() == Qt.Key_Right):
                self.steerTimer.start()
                self.keyRightPressed.emit()


if __name__ == '__main__':
    app = QApplication([])
    keyop = AutocarKeyop()
    keyop.show()
    sys.exit(app.exec_())
