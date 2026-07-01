import sys
import random
import subprocess
import cv2
import numpy as np
import time


from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


robotData = {
    "mode": "AUTO",
    "mission": "IDLE",

    "qr": "NONE",
    "target": "NONE",

    "rpm_fl": 0,
    "rpm_fr": 0,
    "rpm_bl": 0,
    "rpm_br": 0,

    "x": 0,
    "y": 0,
    "distance": 0,
    "yaw": 0,

    "progress": 0,

    "mega": True,
    "uno": True,
    "camera": True
}


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("OMNI YOU - MCT DASHBOARD")
        self.showMaximized()

        self.setStyleSheet("""
            QMainWindow {
                background-color: #121212;
            }

            QFrame {
                background-color: #1E1E1E;
                border: 2px solid #2C2C2C;
                border-radius: 15px;
            }

            QLabel {
                color: white;
                font-size: 16px;
            }
        """)

        self.buildUI()
        self.buildSystemInfo()
        self.buildCamera()
        self.buildRPM()
        self.buildPosition()
        self.buildMap()
        QTimer.singleShot(
            100,
            lambda: self.updateMap(0, 0)
              )
        self.buildBottom()
        self.timer = QTimer()
        self.timer.timeout.connect(self.demoLoop)
        self.timer.start(1000)
        self.cameraTimer = QTimer()
        self.cameraTimer.timeout.connect(
        self.updateCameraFeed
              )
        self.cameraTimer.start(1000)
        self.updateDashboard()

    def updateCameraFeed(self):

     pixmap = QPixmap("frame.jpg")

     if pixmap.isNull():
        return

     pixmap = pixmap.scaled(
        self.cameraLabel.width(),
        self.cameraLabel.height(),
        Qt.KeepAspectRatio,
        Qt.SmoothTransformation
     )

     self.cameraLabel.setPixmap(
        pixmap
     )

    def buildUI(self):

        central = QWidget()
        self.setCentralWidget(central)

        mainLayout = QVBoxLayout(central)
        mainLayout.setSpacing(20)
        mainLayout.setContentsMargins(15, 15, 15, 15)

        # ======================
        # Title
        # ======================

        title = QLabel("OMNI YOU - MCT DASHBOARD")
        title.setAlignment(Qt.AlignCenter)

        title.setStyleSheet("""
            font-size: 28px;
            font-weight: bold;
            color: cyan;
        """)

        mainLayout.addWidget(title)

        # ======================
        # Top Row
        # ======================

        topLayout = QHBoxLayout()

        self.systemFrame = QFrame()
        self.systemFrame.setMinimumWidth(350)

        self.cameraFrame = QFrame()

        topLayout.addWidget(self.systemFrame, 1)
        topLayout.addWidget(self.cameraFrame, 2)

        mainLayout.addLayout(topLayout)

        # ======================
        # RPM Frame


        middleLayout = QHBoxLayout()

        self.rpmFrame = QFrame()
        self.positionFrame = QFrame()

        middleLayout.addWidget(self.rpmFrame, 1)
        middleLayout.addWidget(self.positionFrame, 1)

        mainLayout.addLayout(middleLayout)


        # Bottom Row
   

        bottomLayout = QHBoxLayout()

        self.progressFrame = QFrame()
        self.connectionFrame = QFrame()
        self.logFrame = QFrame()

        bottomLayout.addWidget(self.progressFrame, 1)
        bottomLayout.addWidget(self.connectionFrame, 1)
        bottomLayout.addWidget(self.logFrame, 2)
        self.mapFrame = QFrame()
        self.mapFrame.setMinimumHeight(160)

        mainLayout.addWidget(self.mapFrame)

        mainLayout.addLayout(bottomLayout)



 

    def addTitle(self, frame, text):

        layout = QVBoxLayout(frame)

        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)

        label.setStyleSheet("""
            font-size:20px;
            font-weight:bold;
            color:#00FFFF;
        """)

        layout.addStretch()
        layout.addWidget(label)
        layout.addStretch()

    def buildSystemInfo(self):

      layout = QGridLayout(self.systemFrame)

      title = QLabel("SYSTEM INFO")
      title.setAlignment(Qt.AlignCenter)
      title.setStyleSheet(
        "font-size:20px;"
        "font-weight:bold;"
        "color:cyan;"
      )

      layout.addWidget(title, 0, 0, 1, 2)

      self.modeValue = QLabel("AUTO")
      self.stateValue = QLabel("PLACE_RED")
      self.qrValue = QLabel("RED")
      self.qrValue.setAlignment(Qt.AlignCenter)

      self.qrValue.setStyleSheet("""
    background-color:#8B0000;
    border-radius:15px;
    padding:10px;
    font-size:18px;
    font-weight:bold;
    color:white;
    """)
      self.targetValue = QLabel("RED STATION")
      self.targetValue.setAlignment(Qt.AlignCenter)

      self.targetValue.setStyleSheet("""
    background-color:#8B0000;
    border-radius:15px;
    padding:10px;
    font-size:18px;
    font-weight:bold;
    color:white;
        """)

      layout.addWidget(QLabel("Mode"), 1, 0)
      layout.addWidget(self.modeValue, 1, 1)

      layout.addWidget(QLabel("Mission"), 2, 0)
      layout.addWidget(self.stateValue, 2, 1)

      layout.addWidget(QLabel("QR"), 3, 0)
      layout.addWidget(self.qrValue, 3, 1)

      layout.addWidget(QLabel("Target"), 4, 0)
      layout.addWidget(self.targetValue, 4, 1)

    def buildCamera(self):

     layout = QVBoxLayout(self.cameraFrame)

     title = QLabel("CAMERA FEED")
     title.setAlignment(Qt.AlignCenter)

     title.setStyleSheet(
        "font-size:20px;"
        "font-weight:bold;"
        "color:cyan;"
      )

     self.cameraLabel = QLabel()

     self.cameraLabel.setAlignment(
        Qt.AlignCenter
      )

     self.cameraLabel.setText(
        "LIVE CAMERA FEED"
     )

     self.cameraLabel.setMinimumHeight(220)

     self.cameraLabel.setStyleSheet(
        """
        border:2px solid gray;
        background:#111111;
        font-size:22px;
        """
     )

     layout.addWidget(title)
     layout.addWidget(self.cameraLabel)


    def buildRPM(self):

     layout = QGridLayout(self.rpmFrame)

     title = QLabel("WHEEL SPEEDS")
     title.setAlignment(Qt.AlignCenter)

     title.setStyleSheet(
        "font-size:22px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     layout.addWidget(title, 0, 0, 1, 4)

     self.flRPM = QLabel("120")
     self.frRPM = QLabel("118")
     self.blRPM = QLabel("121")
     self.brRPM = QLabel("119")

     layout.addWidget(QLabel("FL"), 1, 0)
     layout.addWidget(self.flRPM, 1, 1)

     layout.addWidget(QLabel("FR"), 1, 2)
     layout.addWidget(self.frRPM, 1, 3)

     layout.addWidget(QLabel("BL"), 2, 0)
     layout.addWidget(self.blRPM, 2, 1)

     layout.addWidget(QLabel("BR"), 2, 2)
     layout.addWidget(self.brRPM, 2, 3)

    def buildPosition(self):

     layout = QGridLayout(self.positionFrame)

     title = QLabel("POSITION")
     title.setAlignment(Qt.AlignCenter)

     title.setStyleSheet(
        "font-size:20px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     layout.addWidget(title, 0, 0, 1, 2)

     self.xValue = QLabel("114 cm")
     self.yValue = QLabel("140 cm")
     self.distanceValue = QLabel("254 cm")
     self.yawValue = QLabel("178°")

     layout.addWidget(QLabel("X"), 1, 0)
     layout.addWidget(self.xValue, 1, 1)

     layout.addWidget(QLabel("Y"), 2, 0)
     layout.addWidget(self.yValue, 2, 1)

     layout.addWidget(QLabel("Distance"), 3, 0)
     layout.addWidget(self.distanceValue, 3, 1)

     layout.addWidget(QLabel("Yaw"), 4, 0)
     layout.addWidget(self.yawValue, 4, 1)


    def buildBottom(self):

     # Progress
     progressLayout = QVBoxLayout(
        self.progressFrame
     )

     title = QLabel("MISSION PROGRESS")
     title.setAlignment(Qt.AlignCenter)

     title.setStyleSheet(
        "font-size:18px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     self.progressBar = QProgressBar()
     self.progressBar.setValue(70)

     progressLayout.addWidget(title)
     progressLayout.addWidget(
        self.progressBar
     )

     # Connections
     connLayout = QVBoxLayout(
        self.connectionFrame
     )

     title2 = QLabel("CONNECTIONS")
     title2.setAlignment(Qt.AlignCenter)

     title2.setStyleSheet(
        "font-size:18px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     self.megaLabel = QLabel(
        "🟢 Mega"
     )

     self.unoLabel = QLabel(
        "🟢 Uno"
     )

     self.cameraStatus = QLabel(
        "🟢 Camera"
     )

     connLayout.addWidget(title2)
     connLayout.addWidget(
        self.megaLabel
     )
     connLayout.addWidget(
        self.unoLabel
     )
     connLayout.addWidget(
        self.cameraStatus
     )

     # Event Log
     logLayout = QVBoxLayout(
        self.logFrame
     )

     title3 = QLabel("EVENT LOG")
     title3.setAlignment(Qt.AlignCenter)

     title3.setStyleSheet(
        "font-size:18px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     self.logBox = QTextEdit()

     self.logBox.setReadOnly(True)

     self.logBox.append(
        "12:45  QR RED"
     )

     self.logBox.append(
        "12:46  PICK RED"
     )

     self.logBox.append(
        "12:50  AUTO START"
     )

     logLayout.addWidget(title3)
     logLayout.addWidget(
        self.logBox
     )
    def createRPMCard(self, title, value):

     frame = QFrame()

     layout = QVBoxLayout(frame)

     label1 = QLabel(title)
     label1.setAlignment(Qt.AlignCenter)

     label2 = QLabel(value)
     label2.setAlignment(Qt.AlignCenter)

     label3 = QLabel("RPM")
     label3.setAlignment(Qt.AlignCenter)

     label1.setStyleSheet(
        "font-size:20px;"
        "font-weight:bold;"
        "color:cyan;"
     )

     label2.setStyleSheet(
        "font-size:34px;"
        "font-weight:bold;"
        "color:white;"
    )

     label3.setStyleSheet(
        "font-size:16px;"
        "color:gray;"
    )

     layout.addWidget(label1)
     layout.addWidget(label2)
     layout.addWidget(label3)

     return frame
    
   
    
    def buildMap(self):

     layout = QVBoxLayout(self.mapFrame)

     title = QLabel("ARENA MAP")
     title.setAlignment(Qt.AlignCenter)

     title.setStyleSheet("""
        font-size:20px;
        font-weight:bold;
        color:cyan;
     """)

     layout.addWidget(title)

     self.mapLabel = QLabel()

     self.mapLabel.setMinimumHeight(180)
     self.mapLabel.setAlignment(Qt.AlignCenter)

     self.mapLabel.setStyleSheet("""
        background:#111111;
        border:2px solid gray;
        border-radius:15px;
     """)

     layout.addWidget(self.mapLabel)
    
    def updateMap(self, x, y):

     pixmap = QPixmap(
        self.mapLabel.width(),
        self.mapLabel.height()
        )
     pixmap.fill(QColor("#111111"))

     painter = QPainter(pixmap)

     painter.setPen(QPen(Qt.gray, 2))
     w = pixmap.width()
     h = pixmap.height()

     painter.drawRect(
     5,
     5,
     w - 10,
     h - 10
       ) 

     mapX = int((x / 300) * (w - 45))
     mapY = int((y / 200) *(h - 45))
     mapX = max(15, min(mapX, w - 45))
     mapY = max(15, min(mapY, h - 45)) 

     painter.setBrush(QBrush(Qt.red))
     painter.drawEllipse(
        mapX,
        mapY,
        30,
        30
     )

     painter.end()

     self.mapLabel.setPixmap(pixmap)
    

    def updateDashboard(self):

     self.modeValue.setText(
        robotData["mode"]
     )

     self.stateValue.setText(
        robotData["mission"]
     )

     self.flRPM.setText(
        str(robotData["rpm_fl"])
     )

     self.frRPM.setText(
        str(robotData["rpm_fr"])
     )

     self.blRPM.setText(
        str(robotData["rpm_bl"])
     )

     self.brRPM.setText(
        str(robotData["rpm_br"])
     )

     self.xValue.setText(
        f'{robotData["x"]} cm'
     )

     self.yValue.setText(
        f'{robotData["y"]} cm'
     )

     self.distanceValue.setText(
        f'{robotData["distance"]} cm'
     )

     self.yawValue.setText(
        f'{robotData["yaw"]}°'
     )

     self.progressBar.setValue(
        robotData["progress"]
     )
     # ===== QR & Target =====

     qr = robotData["qr"]

     self.qrValue.setText(qr)

     if qr == "NONE":
        self.targetValue.setText(
        "NO TARGET"
         )
     else:
         self.targetValue.setText(
        f"{qr} STATION"
             )

     if qr == "RED":
      color = "#8B0000"

     elif qr == "GREEN":
      color = "#009900"

     elif qr == "BLUE":
      color = "#0044AA"

     else:
      color = "#444444"

     style = f"""
       background-color:{color};
       border-radius:15px;
       padding:10px;
       font-size:18px;
       font-weight:bold;
       color:white;
        """
     # ===== Connections =====

     if robotData["mega"]:
      self.megaLabel.setText(
        "🟢 Mega Connected"
     )
     else:
      self.megaLabel.setText(
        "🔴 Mega Offline"
       )


     if robotData["uno"]:
       self.unoLabel.setText(
        "🟢 Uno Connected"
       )
     else:
       self.unoLabel.setText(
        "🔴 Uno Offline"
       )


     if robotData["camera"]:
      self.cameraStatus.setText(
        "🟢 Camera Ready"
     )
     else:
       self.cameraStatus.setText(
        "🔴 Camera Offline"
       )

     self.qrValue.setStyleSheet(style)
     self.targetValue.setStyleSheet(style)

     self.updateMap(
      robotData["x"],
      robotData["y"]
       )

    
    def demoLoop(self):

     robotData["rpm_fl"] = random.randint(100,130)
     robotData["rpm_fr"] = random.randint(100,130)
     robotData["rpm_bl"] = random.randint(100,130)
     robotData["rpm_br"] = random.randint(100,130)

     robotData["x"] = random.randint(0,250)
     robotData["y"] = random.randint(0,250)
     robotData["distance"] = random.randint(0,500)
     robotData["yaw"] = random.randint(-180,180)
     states = [
       ("IDLE", 0),
     ("SCAN_QR", 20),
     ("PLACE_RED", 40),
     ("PLACE_GREEN", 70),
     ("MISSION_DONE", 100)
       ]

     state, p = random.choice(states)

     robotData["mission"] = state
     robotData["progress"] = p

     robotData["qr"] = random.choice(
     ["RED", "GREEN", "BLUE"]
      )

     self.updateDashboard()

    def setRobotData(
     self,
     mode,
     mission,
     qr,
     fl,
     fr,
     bl,
     br,
     x,
     y,
     distance,
     yaw
    ):
        robotData["mode"] = mode
        robotData["mission"] = mission
        robotData["qr"] = qr
        robotData["rpm_fl"] = fl
        robotData["rpm_fr"] = fr
        robotData["rpm_bl"] = bl
        robotData["rpm_br"] = br
        robotData["x"] = x
        robotData["y"] = y
        robotData["distance"] = distance
        robotData["yaw"] = yaw

        self.updateDashboard()



app = QApplication(sys.argv)
window = Dashboard()
window.show()
sys.exit(app.exec_())