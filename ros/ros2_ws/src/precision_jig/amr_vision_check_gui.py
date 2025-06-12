import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
import sys
import signal
import os
import json
import subprocess

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton,
    QComboBox, QVBoxLayout, QWidget, QSlider, QHBoxLayout,
    QDialog, QDialogButtonBox, QFormLayout, QInputDialog, QFileDialog
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer

CONFIG_DIR = os.path.join(os.path.dirname(__file__), 'config')
os.makedirs(CONFIG_DIR, exist_ok=True)

def adjust_hsv_gamma(img_bgr, hue_shift=0, sat_scale=1.0, val_scale=1.0, gamma=1.0):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    h, s, v = cv2.split(hsv)
    h = (h + hue_shift) % 180
    s = np.clip(s * sat_scale, 0, 255)
    v = np.clip(v * val_scale, 0, 255)
    hsv_adjusted = cv2.merge([h, s, v]).astype(np.uint8)
    bgr_adjusted = cv2.cvtColor(hsv_adjusted, cv2.COLOR_HSV2BGR)
    inv_gamma = 1.0 / gamma
    lut = np.array([(i / 255.0) ** inv_gamma * 255 for i in range(256)]).astype(np.uint8)
    return cv2.LUT(bgr_adjusted, lut)

class StreamSelector(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Stream Type")
        self.combo = QComboBox()
        self.combo.addItems(["rgb", "depth"])
        button_box = QDialogButtonBox(QDialogButtonBox.Ok)
        button_box.accepted.connect(self.accept)
        layout = QFormLayout()
        layout.addRow("Stream:", self.combo)
        layout.addWidget(button_box)
        self.setLayout(layout)

    def get_selection(self):
        return self.combo.currentText()

class VisionFilterGUI(QMainWindow):
    def __init__(self, stream_type):
        super().__init__()
        self.setWindowTitle("RealSense Viewer")
        self.stream_type = stream_type
        self.original_image = None
        self.processed_image = None
        self.filtered_image = None
        self.binary_image = None
        self.selected_filter = "None"

        self.label_origin = QLabel("Origin", alignment=Qt.AlignCenter)
        self.view_origin = QLabel(alignment=Qt.AlignCenter)
        self.label_filtered = QLabel("Filtered", alignment=Qt.AlignCenter)
        self.view_filtered = QLabel(alignment=Qt.AlignCenter)
        self.label_binary = QLabel("Binary", alignment=Qt.AlignCenter)
        self.view_binary = QLabel(alignment=Qt.AlignCenter)

        for view in [self.view_origin, self.view_filtered, self.view_binary]:
            view.setFixedSize(640, 480)
            view.setStyleSheet("background-color: black;")

        self.filter_combo = QComboBox()
        self.filter_combo.addItems([
            "None", "Gaussian", "Median", "Bilateral", "Inpaint",
            "BoxBlur", "Sharpen", "Erosion", "Dilation", "Canny"
        ])
        self.filter_combo.currentTextChanged.connect(self.apply_filter)

        self.hue_slider = QSlider(Qt.Horizontal); self.hue_slider.setRange(-90, 90); self.hue_slider.setValue(0)
        self.sat_slider = QSlider(Qt.Horizontal); self.sat_slider.setRange(10, 300); self.sat_slider.setValue(100)
        self.val_slider = QSlider(Qt.Horizontal); self.val_slider.setRange(10, 300); self.val_slider.setValue(100)
        self.gamma_slider = QSlider(Qt.Horizontal); self.gamma_slider.setRange(10, 300); self.gamma_slider.setValue(100)

        self.save_button = QPushButton("Save Settings"); self.save_button.clicked.connect(self.save_settings)
        self.load_button = QPushButton("Load Settings"); self.load_button.clicked.connect(self.load_settings)
        self.reset_button = QPushButton("Reset Settings"); self.reset_button.clicked.connect(self.reset_settings)

        hsv_layout = QVBoxLayout()
        for name, slider in [("Hue", self.hue_slider), ("Saturation %", self.sat_slider),
                             ("Value %", self.val_slider), ("Gamma %", self.gamma_slider)]:
            hsv_layout.addWidget(QLabel(name)); hsv_layout.addWidget(slider)

        image_layout = QHBoxLayout()
        for label, view in [(self.label_origin, self.view_origin),
                            (self.label_filtered, self.view_filtered),
                            (self.label_binary, self.view_binary)]:
            col = QVBoxLayout(); col.addWidget(label); col.addWidget(view); image_layout.addLayout(col)

        self.controls_layout = QVBoxLayout()
        self.controls_layout.addLayout(image_layout)
        self.controls_layout.addWidget(self.filter_combo)
        if self.stream_type == "rgb":
            self.controls_layout.addLayout(hsv_layout)
            self.controls_layout.addWidget(self.save_button)
            self.controls_layout.addWidget(self.load_button)
            self.controls_layout.addWidget(self.reset_button)

        container = QWidget(); container.setLayout(self.controls_layout)
        self.setCentralWidget(container)

        self.timer = QTimer(); self.timer.timeout.connect(self.update_image); self.timer.start(33)

        self.measure_button = QPushButton("Start Vision Check")
        self.measure_button.clicked.connect(self.open_error_check_window)
        self.controls_layout.addWidget(self.measure_button)

    def set_image(self, img):
        self.original_image = img

    def apply_filter(self):
        self.selected_filter = self.filter_combo.currentText()

    def reset_settings(self):
        self.hue_slider.setValue(0)
        self.sat_slider.setValue(100)
        self.val_slider.setValue(100)
        self.gamma_slider.setValue(100)
        idx = self.filter_combo.findText("None")
        if idx != -1:
            self.filter_combo.setCurrentIndex(idx)

    def get_filtered_image(self):
        return self.filtered_image

    def update_image(self):
        if self.original_image is None:
            return

        img = self.original_image.copy()
        if self.stream_type == "rgb":
            img = adjust_hsv_gamma(
                img,
                self.hue_slider.value(),
                self.sat_slider.value() / 100.0,
                self.val_slider.value() / 100.0,
                self.gamma_slider.value() / 100.0
            )

        self.processed_image = img.copy()
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

        filtered = {
            "Gaussian": lambda x: cv2.GaussianBlur(x, (5, 5), 0),
            "Median": lambda x: cv2.medianBlur(x, 5),
            "Bilateral": lambda x: cv2.bilateralFilter(x, 9, 75, 75),
            "Inpaint": lambda x: cv2.inpaint(x, np.uint8((x == 0) * 255), 2, cv2.INPAINT_TELEA),
            "BoxBlur": lambda x: cv2.blur(x, (5, 5)),
            "Sharpen": lambda x: cv2.filter2D(x, -1, np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])),
            "Erosion": lambda x: cv2.erode(x, np.ones((3, 3), np.uint8), iterations=1),
            "Dilation": lambda x: cv2.dilate(x, np.ones((3, 3), np.uint8), iterations=1),
            "Canny": lambda x: cv2.Canny(x, 100, 200)
        }.get(self.selected_filter, lambda x: x)(img_gray)

        self.filtered_image = filtered
        self.binary_image = np.where(filtered < 128, 0, 255).astype(np.uint8)

        def show(label, img, fmt):
            label.setPixmap(QPixmap.fromImage(QImage(img.data, img.shape[1], img.shape[0], img.strides[0], fmt)))

        show(self.view_origin, self.processed_image, QImage.Format_BGR888)
        show(self.view_filtered, self.filtered_image, QImage.Format_Grayscale8)
        show(self.view_binary, self.binary_image, QImage.Format_Grayscale8)

    def save_settings(self):
        name, ok = QInputDialog.getText(self, "Save Settings", "Enter filename:")
        if ok and name:
            settings = {
                "hue": self.hue_slider.value(),
                "saturation": self.sat_slider.value(),
                "value": self.val_slider.value(),
                "gamma": self.gamma_slider.value(),
                "filter": self.filter_combo.currentText()
            }
            path = os.path.join(CONFIG_DIR, f"{name}.json")
            with open(path, "w") as f:
                json.dump(settings, f, indent=4)

    def load_settings(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Settings", CONFIG_DIR, "JSON Files (*.json)")
        if path:
            with open(path, "r") as f:
                settings = json.load(f)
            self.hue_slider.setValue(settings.get("hue", 0))
            self.sat_slider.setValue(settings.get("saturation", 100))
            self.val_slider.setValue(settings.get("value", 100))
            self.gamma_slider.setValue(settings.get("gamma", 100))
            idx = self.filter_combo.findText(settings.get("filter", "None"))
            if idx != -1:
                self.filter_combo.setCurrentIndex(idx)

    def open_error_check_window(self):
        try:
            subprocess.Popen(["ros2", "run", "precision_jig", "amr_vision_check"])
            print("[INFO] Vision check window launched.")
        except Exception as e:
            print(f"[ERROR] Failed to launch Vision check window: {e}")

class StreamSubscriber(Node):
    def __init__(self, gui: VisionFilterGUI, stream_type: str):
        super().__init__('stream_subscriber_node')
        self.gui = gui
        self.cv_bridge = CvBridge()

        self.encoding = 'bgr8' if stream_type == "rgb" else 'passthrough'
        topic = '/camera/camera/color/image_raw' if stream_type == "rgb" else '/camera/camera/aligned_depth_to_color/image_raw'
        
        self.subscription = self.create_subscription(
                                Image,
                                topic,
                                self.image_callback,
                                10
                            )
        
        self.publisher = self.create_publisher(Image, '/filtered/binary_image', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_filtered_image)

    def image_callback(self, msg):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")
            return

        if self.encoding != 'bgr8':
            img = np.nan_to_num(img, nan=0.0, posinf=0.0, neginf=0.0)
            if img.dtype != np.uint8:
                img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        self.gui.set_image(img)

    def publish_filtered_image(self):
        img = self.gui.binary_image
        if img is not None:
            msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='mono8')
            self.publisher.publish(msg)

def main():
    rclpy.init()
    app = QApplication([])

    selector = StreamSelector()
    if selector.exec_() != QDialog.Accepted:
        print("Stream selection cancelled.")
        return

    stream_type = selector.get_selection()
    gui = VisionFilterGUI(stream_type)
    gui.show()

    node = StreamSubscriber(gui, stream_type)

    def spin_once():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)

    spin_timer = QTimer()
    spin_timer.timeout.connect(spin_once)
    spin_timer.start(10)

    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())
    exit_code = app.exec()

    spin_timer.stop()
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()