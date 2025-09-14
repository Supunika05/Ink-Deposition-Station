import json
import sys
import threading
import time
import cv2
import numpy as np
import logging
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
import AWS_Client
from MainWindow import Ui_MainWindow


# ==========================
#  Persistent AWS worker (runs inside a QThread)
# ==========================
class AwsWorker(QObject):
    feedback = pyqtSignal(str)             # emits status messages back to GUI
    send_request = pyqtSignal(str, str)    # (topic, command)

    def __init__(self, client, mqtt_lock):
        super().__init__()
        self.client = client
        self.mqtt_lock = mqtt_lock
        # connect signal to handler (will run in worker thread)
        self.send_request.connect(self.handle_send, Qt.QueuedConnection)

    def handle_send(self, topic: str, command: str):
        """Handle publish requests (topic, command). Runs in worker thread."""
        message = {
            "timestamp": int(time.time()),
            "command": command
        }
        try:
            with self.mqtt_lock:
                # publish; adapt QoS/retain as needed
                self.client.publish(topic, json.dumps(message), 0)
                self.feedback.emit(f"Sent: {command} → {topic}")
        except Exception as e:
            logging.error(f"Failed to publish: {command} | Error: {e}", exc_info=True)
            self.feedback.emit(f"Error sending {command} → {topic}")


# ==========================
#  Main Window Controller
# ==========================
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # MQTT lock and persistent AWS worker/thread
        self.mqtt_lock = threading.Lock()
        self.aws_thread = QThread()
        self.aws_worker = AwsWorker(AWS_Client.client, self.mqtt_lock)
        self.aws_worker.moveToThread(self.aws_thread)
        # connect worker feedback to a UI handler
        self.aws_worker.feedback.connect(self.update_status)
        # start the thread (worker will respond to send_request emits)
        self.aws_thread.start()

        self.process_running = False

        self._setup_spinboxes()
        self._setup_syringe_boxes()
        self._setup_connections()
        # no serial ports in this refactor; remove or re-add if needed
        # self._populate_com_ports()
        
        self._update_jog_buttons_enabled()


    # ----------------------
    #  UI Setup Helpers
    # ----------------------
    def _setup_spinboxes(self):
        for i in range(1, 5):
            sb = getattr(self, f"flowratedoubleSpinBox{i:02d}", None)
            if sb:
                sb.setRange(0.0, 40.0)
                sb.setSingleStep(0.1)
                sb.setDecimals(1)
                sb.setKeyboardTracking(False)

        if hasattr(self, "coaterspeedoubleSpinBox"):
            self.coaterspeedoubleSpinBox.setRange(0.0, 10.0)
            self.coaterspeedoubleSpinBox.setSingleStep(0.1)
            self.coaterspeedoubleSpinBox.setDecimals(1)
            self.coaterspeedoubleSpinBox.setKeyboardTracking(False)

    def _setup_syringe_boxes(self):
        for i in range(1, 5):
            cb = getattr(self, f"syringersizecomboBox{i:02d}", None)
            if cb:
                cb.clear()
                cb.addItem("10 mL")
                cb.setCurrentIndex(0)
                cb.setEnabled(False)

    # ----------------------
    #  Wiring & UI Handlers
    # ----------------------
    def _setup_connections(self):
        # Ink manual load buttons (fixed RPM handled in firmware)
        self.inkmotorpushButton01Forward.clicked.connect(lambda: self._run_ink_load(1, "FORWARD"))
        self.inkmotorpushButton01Backward.clicked.connect(lambda: self._run_ink_load(1, "BACKWARD"))
        self.inkmotorpushButton01Stop.clicked.connect(lambda: self._stop_ink(1))

        self.inkmotorpushButton02Forward.clicked.connect(lambda: self._run_ink_load(2, "FORWARD"))
        self.inkmotorpushButton02Backward.clicked.connect(lambda: self._run_ink_load(2, "BACKWARD"))
        self.inkmotorpushButton02Stop.clicked.connect(lambda: self._stop_ink(2))

        self.inkmotorpushButton03Forward.clicked.connect(lambda: self._run_ink_load(3, "FORWARD"))
        self.inkmotorpushButton03Backward.clicked.connect(lambda: self._run_ink_load(3, "BACKWARD"))
        self.inkmotorpushButton03Stop.clicked.connect(lambda: self._stop_ink(3))

        self.inkmotorpushButton04Forward.clicked.connect(lambda: self._run_ink_load(4, "FORWARD"))
        self.inkmotorpushButton03Backward_2.clicked.connect(lambda: self._run_ink_load(4, "BACKWARD"))
        self.inkmotorpushButton04Stop.clicked.connect(lambda: self._stop_ink(4))

        # Drive jog buttons (disabled during process)
        self.drivemotorforwardpushButton.clicked.connect(lambda: self._send_drive_jog("FORWARD"))
        self.drivemotorbackwardpushButton.clicked.connect(lambda: self._send_drive_jog("BACKWARD"))
        self.drivemotorstoppushButton.clicked.connect(lambda: self._send_drive_jog("STOP"))

        # Flow rate setpoints → process speeds (mL/min)
        self.flowratedoubleSpinBox01.valueChanged.connect(lambda _: self._send_ink_setpoint(1))
        self.flowratedoubleSpinBox02.valueChanged.connect(lambda _: self._send_ink_setpoint(2))
        self.flowratedoubleSpinBox03.valueChanged.connect(lambda _: self._send_ink_setpoint(3))
        self.flowratedoubleSpinBox04.valueChanged.connect(lambda _: self._send_ink_setpoint(4))

        # Coater/drive process speed (m/min)
        self.coaterspeedoubleSpinBox.valueChanged.connect(self._send_drive_setpoint)

        # Camera
        self.pushButton.clicked.connect(self.CameraOn)
        self.pushButton_2.clicked.connect(self.CameraOff)

        # Master process buttons
        self.startpushButton.clicked.connect(self.start_process)
        self.stoppushButton.clicked.connect(self.stop_process)

        # Subscribe to video frames via AWS client (unchanged)
        try:
            AWS_Client.client.subscribe(AWS_Client.topic_video, 1, self.onFramesReceived)
            print(f"[INFO] Subscribed to topic: {AWS_Client.topic_video}")
        except Exception as e:
            print(f"[WARN] Failed to subscribe to video topic: {e}")

    # ----------------------
    #  Connection checker
    # ----------------------
    def _is_connected(self) -> bool:
        # return True if client object exists.
        try:
            return AWS_Client.client is not None
        except Exception:
            return False

    # ----------------------
    #  Ink (manual load + process setpoints)
    # ----------------------
    def _send_ink_setpoint(self, pump_number: int):
        if not self._is_connected():
            return
        sb = getattr(self, f"flowratedoubleSpinBox{pump_number:02d}")
        value = float(sb.value())
        # emit (topic, command)
        self.aws_worker.send_request.emit(AWS_Client.topic_command, f"INK{pump_number:02d}_SPEED {value}")

    def _run_ink_load(self, pump_number: int, direction: str):
        if not self._is_connected():
            self.update_status(f"Cannot run INK{pump_number:02d}: not connected.")
            return
        self.aws_worker.send_request.emit(AWS_Client.topic_command, f"INK{pump_number:02d} {direction}")
        self._set_ink_indicator(pump_number, True)

    def _stop_ink(self, pump_number: int):
        if not self._is_connected():
            self.update_status(f"Cannot stop INK{pump_number:02d}: not connected.")
            return
        self.aws_worker.send_request.emit(AWS_Client.topic_command, f"INK{pump_number:02d} STOP")
        self._set_ink_indicator(pump_number, False)

    def _set_ink_indicator(self, pump_number: int, running: bool):
        indicator = getattr(self, f"ink{pump_number:02d}indicator", None)
        if indicator is not None:
            style = (
                "QLabel { background: #27ae60; border-radius: 7px; }"
                if running
                else "QLabel { background: #e74c3c; border-radius: 7px; }"
            )
            indicator.setStyleSheet(style)

    # ----------------------
    #  Drive (process setpoint + jog)
    # ----------------------
    def _send_drive_setpoint(self):
        if not self._is_connected():
            return
        value = float(self.coaterspeedoubleSpinBox.value())
        self.aws_worker.send_request.emit(AWS_Client.topic_command, f"DRIVE_SPEED {value}")

    def _send_drive_jog(self, direction: str):
        if not self._is_connected():
            self.update_status("Cannot control drive: not connected.")
            return
        if self.process_running:
            self.update_status("Drive jog disabled during process.")
            return
        self.aws_worker.send_request.emit(AWS_Client.topic_command, f"DRIVE {direction}")

    def _update_jog_buttons_enabled(self):
        enable = (not self.process_running)
        for btn in (
            self.drivemotorforwardpushButton,
            self.drivemotorbackwardpushButton,
            self.drivemotorstoppushButton,
        ):
            btn.setEnabled(enable)

    # ----------------------
    #  Master Process
    # ----------------------
    def start_process(self):
        if not self._is_connected():
            self.update_status("Cannot start: not connected to AWS.")
            return

        # Re-send setpoints (safe)
        self._send_drive_setpoint()
        for i in range(1, 5):
            self._send_ink_setpoint(i)

        # Start process
        self.aws_worker.send_request.emit(AWS_Client.topic_command, "PROCESS START")
        self.process_running = True
        self._update_jog_buttons_enabled()

        if hasattr(self, "currentlabel01"):
            self.currentlabel01.setText("Running")
        if hasattr(self, "currentlabel02"):
            self.currentlabel02.setText("Ready")

    def stop_process(self):
        if self._is_connected():
            self.aws_worker.send_request.emit(AWS_Client.topic_command, "PROCESS STOP")

        self.process_running = False
        self._update_jog_buttons_enabled()

        for i in range(1, 5):
            self._set_ink_indicator(i, False)

        if hasattr(self, "currentlabel01"):
            self.currentlabel01.setText("Stopped")
        if hasattr(self, "currentlabel02"):
            self.currentlabel02.setText("Ready")

    # -------------------- Video frame handler --------------------
    def onFramesReceived(self, client, userdata, message):
        try:
            nparr = np.frombuffer(message.payload, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                if hasattr(self, "livecamlabel"):
                    self.livecamlabel.setPixmap(
                        pixmap.scaled(
                            self.livecamlabel.width(),
                            self.livecamlabel.height(),
                            Qt.KeepAspectRatio,
                            Qt.SmoothTransformation
                        )
                    )
            else:
                print("[WARN] Failed to decode image frame")
        except Exception as e:
            print(f"[ERROR] Exception while decoding image: {e}")

    # -------------------- Camera controls --------------------
    def CameraOn(self):
        self.aws_worker.send_request.emit(AWS_Client.topic_cam, "ON")

    def CameraOff(self):
        self.aws_worker.send_request.emit(AWS_Client.topic_cam, "OFF")

    # ----------------------
    #  Status update helper
    # ----------------------
    def update_status(self, message: str):
        print(f"AWS: {message}")

    # ----------------------
    #  Qt Lifecycle
    # ----------------------
    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            # Signal thread to quit and wait for it to finish
            try:
                self.aws_thread.quit()
                self.aws_thread.wait(1500)
            except Exception:
                pass

            # send camera off synchronously before exit
            try:
                # send a final camera-off request synchronously (avoid relying on worker)
                AWS_Client.client.publish(AWS_Client.topic_cam, json.dumps({"timestamp": int(time.time()), "command": "OFF"}), 0)
            except Exception:
                pass
        finally:
            event.accept()


# ---------------
#  Application entry
# ---------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
