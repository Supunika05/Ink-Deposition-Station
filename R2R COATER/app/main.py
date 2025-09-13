import json
import sys
import threading
import time
import serial
import serial.tools.list_ports
import cv2
import numpy as np
import logging
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QIcon, QImage, QPixmap
from PyQt5.QtCore import Qt
import AWS_Client
from MainWindow import Ui_MainWindow


# ==========================
#  Background Serial Thread
# ==========================
class SerialThread(QThread):
    feedback = pyqtSignal(str)
    connection_status = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.serial = None
        self.is_running = True
        self.port = None
        self.baudrate = 115200

    def run(self):
        # Continuously poll the serial buffer and emit lines to the GUI
        while self.is_running:
            if self.serial and self.serial.is_open:
                try:
                    if self.serial.in_waiting:
                        data = self.serial.readline().decode(errors="ignore").strip()
                        if data:
                            self.feedback.emit(data)
                except Exception as e:
                    self.feedback.emit(f"Serial error: {str(e)}")
                    self.disconnect()
            QtCore.QThread.msleep(10)

    def connect_port(self, port):
        try:
            self.port = port
            self.serial = serial.Serial(port, self.baudrate, timeout=1)
            self.connection_status.emit(True)
            self.feedback.emit(f"Connected to {port}")
        except Exception as e:
            self.feedback.emit(f"Connection error: {str(e)}")
            self.connection_status.emit(False)

    def disconnect(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        finally:
            self.connection_status.emit(False)
            self.feedback.emit("Disconnected")

    def send_command(self, cmd):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{cmd}\n".encode())
            except Exception as e:
                self.feedback.emit(f"Send error: {str(e)}")
        else:
            self.feedback.emit(f"Not connected. Tried to send: {cmd}")

    def stop(self):
        self.is_running = False
        self.disconnect()


# ==========================
#  Main Window Controller
# ==========================
class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.serial_thread = SerialThread()
        self.process_running = False

        # Initialize MQTT lock
        self.mqtt_lock = threading.Lock()

        self._setup_spinboxes()
        self._setup_syringe_boxes()
        self._setup_connections()
        self._populate_com_ports()
        self._hide_pause_button_if_present()
        self._update_jog_buttons_enabled()

        screen = QApplication.primaryScreen().geometry()
        self.setGeometry(screen)  # resize to screen size
        self.show()


    # ----------------------
    #  UI Setup Helpers
    # ----------------------
    def _setup_spinboxes(self):
        # Flow rates (mL/min): 0..40, step 0.1, 1 decimal, no keyboard tracking to avoid spam
        for i in range(1, 5):
            sb = getattr(self, f"flowratedoubleSpinBox{i:02d}", None)
            if sb:
                sb.setRange(0.0, 40.0)
                sb.setSingleStep(0.1)
                sb.setDecimals(1)
                sb.setKeyboardTracking(False)

        # Drive/coater speed (m/min): 0..10, step 0.1, 1 decimal
        if hasattr(self, "coaterspeedoubleSpinBox"):
            self.coaterspeedoubleSpinBox.setRange(0.0, 10.0)
            self.coaterspeedoubleSpinBox.setSingleStep(0.1)
            self.coaterspeedoubleSpinBox.setDecimals(1)
            self.coaterspeedoubleSpinBox.setKeyboardTracking(False)

    def _setup_syringe_boxes(self):
        # For now, force "10 mL" selection for all injectors and disable editing
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
        # Serial thread signals
        self.serial_thread.feedback.connect(self.update_status)
        self.serial_thread.connection_status.connect(self.update_connection_indicator)

        # COM port
        self.comportconnectpushButton.clicked.connect(self.toggle_connection)
        self.comportrefreshpushButton.clicked.connect(self._populate_com_ports)

        # Ink manual load buttons (fixed 10 RPM handled in firmware)
        self.inkmotorpushButton01Forward.clicked.connect(lambda: self._run_ink_load(1, "FORWARD"))
        self.inkmotorpushButton01Backward.clicked.connect(lambda: self._run_ink_load(1, "BACKWARD"))
        self.inkmotorpushButton01Stop.clicked.connect(lambda: self._stop_ink(1))

        self.inkmotorpushButton02Forward.clicked.connect(lambda: self._run_ink_load(2, "FORWARD"))
        self.inkmotorpushButton02Backward.clicked.connect(lambda: self._run_ink_load(2, "BACKWARD"))
        self.inkmotorpushButton02Stop.clicked.connect(lambda: self._stop_ink(2))

        self.inkmotorpushButton03Forward.clicked.connect(lambda: self._run_ink_load(3, "FORWARD"))
        self.inkmotorpushButton03Backward.clicked.connect(lambda: self._run_ink_load(3, "BACKWARD"))
        self.inkmotorpushButton03Stop.clicked.connect(lambda: self._stop_ink(3))

        # Note: pump 4 backward button name in your UI is "inkmotorpushButton03Backward_2"
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

        # ---------- MQTT subscribe for video frames ----------
        AWS_Client.client.subscribe(AWS_Client.topic_video, 1, self.onFramesReceived)
        print(f"[INFO] Subscribed to topic: {AWS_Client.topic_video}")

    # ----------------------
    #  Serial/UI Utilities
    # ----------------------
    def _populate_com_ports(self):
        self.comportcomboBox.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.comportcomboBox.addItem(port.device)

    def toggle_connection(self):
        if not self.serial_thread.isRunning():
            port = self.comportcomboBox.currentText()
            if port:
                self.serial_thread.start()
                self.serial_thread.connect_port(port)
                self.comportconnectpushButton.setText("Disconnect")
        else:
            self.serial_thread.disconnect()
            self.comportconnectpushButton.setText("Connect")

    def update_status(self, message: str):
        # For now, print to console; you could also append to a QTextEdit if you add one
        print(f"ESP32: {message}")

    def update_connection_indicator(self, connected: bool):
        style = (
            "QLabel { background: #27ae60; border-radius: 7px; }"
            if connected
            else "QLabel { background: #e74c3c; border-radius: 7px; }"
        )
        self.comportindicator.setStyleSheet(style)

    def _hide_pause_button_if_present(self):
        # You decided to remove Pause; hide it defensively if it still exists in the .ui
        if hasattr(self, "pausepushButton"):
            self.pausepushButton.hide()
            self.pausepushButton.setEnabled(False)

    def _is_connected(self) -> bool:
        return bool(self.serial_thread.serial and self.serial_thread.serial.is_open)

    # ----------------------
    #  Ink (manual load + process setpoints)
    # ----------------------
    def _send_ink_setpoint(self, pump_number: int):
        """Send process flow setpoint (mL/min). Does not start the motor."""
        if not self._is_connected():
            return
        sb = getattr(self, f"flowratedoubleSpinBox{pump_number:02d}")
        value = float(sb.value())
        self.serial_thread.send_command(f"INK{pump_number:02d}_SPEED {value}")

    def _run_ink_load(self, pump_number: int, direction: str):
        """Manual loading at fixed RPM (set in firmware)."""
        if not self._is_connected():
            self.update_status(f"Cannot run INK{pump_number:02d}: not connected.")
            return
        self.serial_thread.send_command(f"INK{pump_number:02d} {direction}")
        self._set_ink_indicator(pump_number, True)

    def _stop_ink(self, pump_number: int):
        if not self._is_connected():
            self.update_status(f"Cannot stop INK{pump_number:02d}: not connected.")
            return
        self.serial_thread.send_command(f"INK{pump_number:02d} STOP")
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
        """Store process drive speed (m/min). Does not start the motor."""
        if not self._is_connected():
            return
        value = float(self.coaterspeedoubleSpinBox.value())
        self.serial_thread.send_command(f"DRIVE_SPEED {value}")

    def _send_drive_jog(self, direction: str):
        """Jog at preset speed (in firmware). Disabled during process."""
        if not self._is_connected():
            self.update_status("Cannot control drive: not connected.")
            return
        if self.process_running:
            # Should be disabled; guard anyway.
            self.update_status("Drive jog disabled during process.")
            return
        self.serial_thread.send_command(f"DRIVE {direction}")

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
        """Start process: drive forward at user m/min + injectors with Q>0 forward."""
        if not self._is_connected():
            self.update_status("Cannot start: not connected to a COM port.")
            return

        # Ensure firmware has current setpoints (spinboxes already send, but safe to re-send)
        self._send_drive_setpoint()
        for i in range(1, 5):
            self._send_ink_setpoint(i)

        # Start everything
        self.serial_thread.send_command("PROCESS START")
        self.sendCommandToAWS("START", AWS_Client.topic_command)
        self.process_running = True
        self._update_jog_buttons_enabled()

        # Labels
        if hasattr(self, "currentlabel01"):
            self.currentlabel01.setText("Running")
        if hasattr(self, "currentlabel02"):
            self.currentlabel02.setText("Ready")

    def stop_process(self):
        """Stop everything; DO NOT reset any setpoints."""
        if self._is_connected():
            self.serial_thread.send_command("PROCESS STOP")
            self.sendCommandToAWS("STOP", AWS_Client.topic_command)

        self.process_running = False
        self._update_jog_buttons_enabled()

        # Turn off ink indicators (firmware stopped them)
        for i in range(1, 5):
            self._set_ink_indicator(i, False)

        # Labels
        if hasattr(self, "currentlabel01"):
            self.currentlabel01.setText("Stopped")
        if hasattr(self, "currentlabel02"):
            self.currentlabel02.setText("Ready")

    def sendCommandToAWS(self, command, topic):
        message = {
            "timestamp": int(time.time()),
            "command": command
        }
        try:
            with self.mqtt_lock:
                AWS_Client.client.publish(topic, json.dumps(message), 0)
                print(f"Sent: {command}")
        except Exception as e:
            logging.error(f"Failed to publish command: {command} | Error: {e}", exc_info=True)
            print(f"Error: Failed to send command: {command}")

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
                # Scale smoothly to fit label while keeping aspect ratio
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
        self.sendCommandToAWS("ON", AWS_Client.topic_cam)

    def CameraOff(self):
        self.sendCommandToAWS("OFF", AWS_Client.topic_cam)


    # ----------------------
    #  Qt Lifecycle
    # ----------------------
    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            self.serial_thread.stop()
            self.serial_thread.wait(1000)
            self.CameraOff()
        finally:
            event.accept()

# ---------------
#  Application in
# ---------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())