import sys
import cv2
import random
import math
import numpy as np
from ultralytics import YOLO
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QPushButton, QFrame, QProgressBar)
from PySide6.QtCore import Qt, QTimer, QPoint
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
from pymavlink import mavutil

# --- TAKTİK RADAR (KÜÇÜLTÜLMÜŞ) ---
class TacticalRadar(QWidget):
    def __init__(self):
        super().__init__()
        self.drone_yaw = 0
        self.targets = []
        for _ in range(3):
            self.targets.append([random.randint(-100, 100), random.randint(-50, 50)])

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Arka Plan (Lacivert/Siyah Askeri Ton)
        painter.fillRect(self.rect(), QColor(10, 15, 25))
        w = self.width()
        h = self.height()
        cx, cy = w // 2, h // 2 
        
        # Grid Çizgileri
        pen = QPen(QColor(0, 255, 255, 50))
        pen.setStyle(Qt.DotLine)
        painter.setPen(pen)
        for i in range(0, w, 40): painter.drawLine(i, 0, i, h)
        for i in range(0, h, 40): painter.drawLine(0, i, w, i)

        # Radar Daireleri
        pen = QPen(QColor(0, 255, 0, 150))
        pen.setStyle(Qt.SolidLine)
        painter.setPen(pen)
        painter.drawEllipse(QPoint(cx, cy), 40, 40)
        painter.drawEllipse(QPoint(cx, cy), 80, 80)
        
        # Hedefler
        painter.setBrush(QColor(255, 50, 50))
        painter.setPen(Qt.NoPen)
        for t in self.targets:
            tx, ty = cx + t[0], cy + t[1]
            painter.drawEllipse(QPoint(tx, ty), 5, 5)

        # Drone (Biz)
        painter.translate(cx, cy)
        painter.rotate(self.drone_yaw)
        painter.setBrush(QColor(0, 255, 255))
        path = QPoint(0, -10), QPoint(-8, 8), QPoint(8, 8)
        painter.drawPolygon(path)
        painter.resetTransform()
        
        # Koordinat Yazısı
        painter.setPen(QColor(200, 200, 200))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(5, h-5, "GPS: 40.2015 N / 32.6868 E")

# --- ANA PENCERE ---
class ModernGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TEKNOFEST T-45 SİHA KONSOLU (YOLOv8 AI)")
        self.resize(1280, 850)
        
        # AI Modeli
        print("AI Modeli Başlatılıyor...")
        self.model = YOLO("yolov8n.pt")
        
        self.cap = cv2.VideoCapture(0)
        self.master = None
        self.is_connected = False
        self.yaw_angle = 0 
        
        # Stil
        self.setStyleSheet("""
            QMainWindow { background-color: #0f0f0f; }
            QLabel { color: #e0e0e0; font-family: 'Segoe UI', Arial; font-size: 12px; }
            QFrame { background-color: #1a1a1a; border-radius: 5px; border: 1px solid #333; }
            QProgressBar { 
                border: 1px solid #444; border-radius: 3px; text-align: center; background: #111; color: white;
            }
            QProgressBar::chunk { background-color: #007aff; }
            QPushButton { 
                background-color: #333; color: white; border: 1px solid #555;
                border-radius: 4px; padding: 6px; font-weight: bold;
            }
            QPushButton:hover { background-color: #007aff; border: 1px solid #007aff; }
            QPushButton#danger { background-color: #a00; border: 1px solid #f00; }
        """)

        self.init_ui()
        
        # Zamanlayıcılar
        self.timer_data = QTimer()
        self.timer_data.timeout.connect(self.update_data)
        self.timer_data.start(100)
        
        self.timer_cam = QTimer()
        self.timer_cam.timeout.connect(self.update_video)
        self.timer_cam.start(30)
        
        self.connect_drone()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central) # Ana düzen YATAY (Sol Panel - Orta - Sağ)

        # --- 1. SOL PANEL (TELEMETRİ) ---
        panel_left = QFrame()
        panel_left.setFixedWidth(220)
        layout_l = QVBoxLayout(panel_left)
        
        layout_l.addWidget(QLabel("SİSTEM DURUMU"))
        
        # Bar Göstergeler
        self.bar_alt = self.create_bar(layout_l, "İRTİFA (m)", 150)
        self.bar_spd = self.create_bar(layout_l, "HIZ (m/s)", 30)
        self.bar_bat = self.create_bar(layout_l, "BATARYA (%)", 100)
        
        layout_l.addSpacing(20)
        self.lbl_mode = QLabel("MOD: BEKLEMEDE")
        self.lbl_mode.setStyleSheet("font-size: 16px; font-weight: bold; color: yellow;")
        self.lbl_mode.setAlignment(Qt.AlignCenter)
        layout_l.addWidget(self.lbl_mode)
        
        layout_l.addStretch()
        
        # --- 2. ORTA PANEL (BÜYÜK KAMERA + KÜÇÜK HARİTA) ---
        panel_center = QWidget()
        layout_c = QVBoxLayout(panel_center)
        layout_c.setContentsMargins(0,0,0,0)
        
        # A. Harita/Radar (Üstte ama daha ince - %25)
        self.radar = TacticalRadar()
        self.radar.setFixedHeight(200) # Sabit Yükseklik
        
        # B. Kamera (Altta ve Büyük - %75)
        self.frm_cam = QFrame()
        self.frm_cam.setStyleSheet("background: black; border: 2px solid #007aff;")
        lay_cam = QVBoxLayout(self.frm_cam)
        lay_cam.setContentsMargins(0,0,0,0)
        
        self.lbl_video = QLabel("KAMERA SİNYALİ YOK")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setScaledContents(True)
        lay_cam.addWidget(self.lbl_video)
        
        layout_c.addWidget(self.radar)
        layout_c.addWidget(self.frm_cam, 1) # 1 = Esnek, alanı kapla

        # --- 3. SAĞ PANEL (KOMUTLAR) ---
        panel_right = QFrame()
        panel_right.setFixedWidth(160)
        layout_r = QVBoxLayout(panel_right)
        
        layout_r.addWidget(QLabel("KOMUTLAR"))
        layout_r.addWidget(QPushButton("BAĞLAN"))
        layout_r.addWidget(QPushButton("ARM"))
        layout_r.addWidget(QPushButton("KALKIŞ"))
        layout_r.addSpacing(10)
        layout_r.addWidget(QPushButton("MOD: VTOL"))
        layout_r.addWidget(QPushButton("MOD: UÇAK"))
        layout_r.addStretch()
        
        btn_lock = QPushButton("KİLİTLE")
        btn_lock.setObjectName("danger")
        btn_lock.setFixedHeight(50)
        layout_r.addWidget(btn_lock)

        # Yerleşim
        layout.addWidget(panel_left)
        layout.addWidget(panel_center, 1)
        layout.addWidget(panel_right)

    def create_bar(self, layout, title, max_val):
        layout.addWidget(QLabel(title))
        bar = QProgressBar()
        bar.setRange(0, max_val)
        bar.setValue(0)
        bar.setFixedHeight(20)
        layout.addWidget(bar)
        return bar

    def update_video(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # YOLO Tespiti
                results = self.model(frame, verbose=False, stream=True)
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls_name = self.model.names[int(box.cls[0])]
                        conf = float(box.conf[0])
                        
                        # Kutu Rengi
                        color = (0, 255, 0)
                        if cls_name == "traffic light": color = (0, 255, 255)
                        elif cls_name == "person": color = (255, 0, 0)
                        
                        if conf > 0.4:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"{cls_name} %{int(conf*100)}", 
                                      (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # PySide'a Çevir
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                qt_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
                self.lbl_video.setPixmap(QPixmap.fromImage(qt_img))

    def update_data(self):
        # Radar Hareketi
        self.yaw_angle = (self.yaw_angle + 2) % 360
        self.radar.drone_yaw = self.yaw_angle
        self.radar.update()
        
        # Telemetri (Simülasyon veya Gerçek)
        alt, spd, bat = 0, 0, 0
        if self.is_connected and self.master:
            msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if msg:
                alt = msg.alt
                spd = msg.groundspeed
                bat = 95 # Örnek
        else:
            # Demo verisi
            alt = int(random.uniform(50, 60))
            spd = int(random.uniform(15, 20))
            bat = int(random.uniform(80, 85))
            self.lbl_mode.setText("OTO. ARAMA")
            
        self.bar_alt.setValue(alt)
        self.bar_spd.setValue(spd)
        self.bar_bat.setValue(bat)

    def connect_drone(self):
        try:
            self.master = mavutil.mavlink_connection('udpin:localhost:14550')
            self.is_connected = True
        except:
            self.is_connected = False
            
    def closeEvent(self, event):
        self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ModernGCS()
    window.show()
    sys.exit(app.exec())