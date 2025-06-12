import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge
import numpy as np

import os
import sys
import json
import signal

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QListWidget,
    QVBoxLayout, QHBoxLayout, QWidget, QSizePolicy, QPushButton, QInputDialog
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer

CONFIG_DIR = os.path.join(os.path.dirname(__file__), 'result')
os.makedirs(CONFIG_DIR, exist_ok=True)

class AmrVisionCheck(Node):
    def __init__(self, window):
        super().__init__('amr_vision_check_node')
        self.window = window
        self.bridge = CvBridge()
        self.camera_info_ready = False
        self.Z_mm = 1950.0
        self.MIN_AREA = 500

        self.fx = self.fy = self.cx = self.cy = 0.0
        self.current_info_text = ""

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_img_callback,
            10
        )

        self.create_subscription(
            Image,
            '/filtered/binary_image',
            self.binary_img_callback,
            10
        )

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_ready = True

    def color_img_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.rotate(img, cv2.ROTATE_180)
            self.window.set_origin_image(img)
        except Exception as e:
            self.get_logger().error(f"[ERROR] Origin image conversion: {e}")

    def binary_img_callback(self, msg):
        if not self.camera_info_ready:
            return
        try:
            binary = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            binary = cv2.rotate(binary, cv2.ROTATE_180)
            display = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

            cx, cy = int(self.cx), int(self.cy)
            cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)

            contours, _ = cv2.findContours(cv2.bitwise_not(binary), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_AREA]
            self.current_info_text = ""

            if valid:
                largest = max(valid, key=cv2.contourArea)
                rect = cv2.minAreaRect(largest)
                box = np.intp(cv2.boxPoints(rect))
                cv2.drawContours(display, [box], 0, (0, 255, 0), 2)

                (bx, by) = rect[0]
                bx, by = int(bx), int(by)
                cv2.circle(display, (bx, by), 5, (0, 0, 255), -1)

                dx_mm = round((bx - self.cx) * self.Z_mm / self.fx, 2)
                dy_mm = round((by - self.cy) * self.Z_mm / self.fy, 2)
                angle = rect[2] if rect[1][0] < rect[1][1] else rect[2] + 90
                angle = round(angle, 2)

                self.current_info_text = f"Offset: X={dx_mm}mm, Y={dy_mm}mm | Angle: {angle}°"
                cv2.putText(display, self.current_info_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2, cv2.LINE_AA)

            self.window.set_binary_image(display)
        except Exception as e:
            self.get_logger().error(f"[ERROR] Binary image conversion: {e}")

class AmrVisionCheckWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Vision Check Result")

        self.origin_label = QLabel("Origin Image")
        self.binary_label = QLabel("Binary Image")
        self.result_list = QListWidget()
        self.result_stats = QLabel()
        self.save_button = QPushButton("Save Result")
        self.save_button.clicked.connect(self.save_results)
        self.saved_offsets = []

        for label in [self.origin_label, self.binary_label]:
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("background-color: black;")
            label.setScaledContents(True)

        image_layout = QVBoxLayout()
        image_layout.addWidget(self.binary_label)
        image_layout.addWidget(self.origin_label)

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.result_list)
        right_layout.addWidget(self.result_stats)
        right_layout.addWidget(self.save_button)

        main_layout = QHBoxLayout()
        main_layout.addLayout(image_layout)
        main_layout.addLayout(right_layout)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        rclpy.init()
        self.node = AmrVisionCheck(self)

        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self.timer.start(30)

    def set_origin_image(self, img):
        self._update_image(self.origin_label, img)

    def set_binary_image(self, img):
        self._update_image(self.binary_label, img)

    def _update_image(self, label, img):
        h, w = label.height(), label.width()
        qimg = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qimg).scaled(w, h, Qt.KeepAspectRatio)
        label.setPixmap(pixmap)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            text = self.node.current_info_text
            if text:
                count = len(self.saved_offsets) + 1
                self.saved_offsets.append(text)
                self.result_list.addItem(f"[{count}] {text}")
                self.update_stats()

    def update_stats(self):
        dxs, dys, angles = [], [], []
        for entry in self.saved_offsets:
            try:
                dx = float(entry.split("X=")[1].split("mm")[0])
                dy = float(entry.split("Y=")[1].split("mm")[0])
                ang = float(entry.split("Angle: ")[1].split("°")[0])
                dxs.append(dx)
                dys.append(dy)
                angles.append(ang)
            except Exception:
                continue

        if dxs:
            self.result_stats.setText(
                f"X(mm): min={min(dxs)} max={max(dxs)} avg={round(sum(dxs)/len(dxs), 2)} | "
                f"Y(mm): min={min(dys)} max={max(dys)} avg={round(sum(dys)/len(dys), 2)} | "
                f"Angle(°): min={min(angles)} max={max(angles)} avg={round(sum(angles)/len(angles), 2)}"
            )

    def save_results(self):
        name, ok = QInputDialog.getText(self, "Save Result", "Enter file name:")
        if ok and name:
            data = {"results": self.saved_offsets}
            path = os.path.join(CONFIG_DIR, f"{name}.json")
            with open(path, "w") as f:
                json.dump(data, f, indent=4)
            print(f"[INFO] Results saved to {path}")

    def closeEvent(self, event):
        self.timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main(args=None):
    app = QApplication(sys.argv)
    window = AmrVisionCheckWindow()
    window.show()

    def handle_sigint(sig, frame):
        print("[INFO] Ctrl+C detected, closing application...")
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)
    sys.exit(app.exec())

if __name__ == '__main__':
    main()