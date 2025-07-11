import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout,
    QComboBox, QLabel, QFileDialog, QHBoxLayout, 
    QCheckBox, QSpacerItem, QSizePolicy, QStatusBar
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.ticker import AutoMinorLocator
from collections import deque
import csv
import datetime
import numpy as np
from numpy import array, polyfit, exp

class AltimeterSpeed():
    """Stores time, depth pairs in a fifo containing listlen
    pairs. Fits the istantaneous descent (negative) or ascent
    (positive) speed at the time of the last inserted pair through a
    polynomial of order polydegree with weight exponentially
    decaying with time scale tau. Also computes the average speed
    over the last listlen datapoints
    """
    def __init__(self, polydegree=3, listlen=40, tau=2):
        """See the class help for the meaning of the parameters.
        """
        self.polydegree = polydegree
        self.listlen = listlen
        self.tau = tau #exponential time scale; must match the time
                       #units of the data.
        self.times = []
        self.depths = []
        
    def set_time_depth(self, time, depth):
        """Stores the time, depth pair in the fifo. Returns nothing.
        """
        self.times.append(time)
        self.depths.append(depth)
        if len(self.times) > self.listlen:
            self.depths.pop(0)
            self.times.pop(0)
            
    def get_speeds(self):
        """Returns ispeed, aspeed : a 'istantaneous' speed (e.g. speed
        on time scales shorter than the parameter tau used in init),
        and an 'average' speed, that is the average speed over
        'listlen' datapoints. Descent speed is negative, ascent is
        positive.

        The istantaneous speed refers to the time specified in the
        last call of "set_time_depth". The average speed is best
        attributed to the mid time of the fifo dataset. The units of
        both speeds are determined by the units of time and depth used
        in "set_time_depth". E.g. if time is in seconds and time is in
        meters, the speed will be in meters/second.

        If the fifo doesn't contain at least two datapoints, then the
        average speed returned is zero. If the fifo doesn't contain at
        least self.polydegree+1 pairs, then the instantaneous speed
        returned is 0. Thus one must call "set_time_depth" at least
        (2) self.polydegree+1 times before obtaining a non-zero
        (average) instantaneous speed.
        """
        if len(self.times) < 2:
            return 0., 0.
        t = array(self.times) - self.times[-1]
        d = array(self.depths)
        if len(self.times) <= self.polydegree:
            ispeed = 0.
        else:
            w = exp(t/self.tau)
            ispeed = polyfit(t, d, self.polydegree, w=w)[-2]
        aspeed = polyfit(t, d, 1)[-2]
        return ispeed, aspeed

class SerialPlotApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🌊 Altimeter Plotter")
        self.setGeometry(300, 150, 1000, 800)
        self.setStyleSheet("""
            background-color: #1e1e2f; 
            color: white;
            QPushButton {
                padding: 5px 10px;
                border-radius: 4px;
                min-width: 80px;
            }
            QComboBox {
                padding: 3px;
            }
        """)

        self.serial = None
        self.data = deque(maxlen=500)  # Increased data buffer
        self.time = deque(maxlen=500)
        self.start_time = datetime.datetime.now()
        self.log_data = []
        self.last_value = None
        self.sampling_rate = 100  # ms
        
        # Initialize speed calculator
        self.speed_calculator = AltimeterSpeed(polydegree=3, listlen=40, tau=2)
        self.speed_history = deque(maxlen=100)  # Store speed values for display

        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.refresh_ports()

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # --- Top Row: COM port selector + refresh button ---
        port_layout = QHBoxLayout()
        port_layout.setSpacing(10)
        
        self.port_label = QLabel("Select COM Port:")
        self.port_label.setFont(QFont("Arial", 10, QFont.Bold))
        port_layout.addWidget(self.port_label)

        self.port_combo = QComboBox()
        self.port_combo.setFont(QFont("Arial", 10))
        self.port_combo.setMinimumWidth(200)
        port_layout.addWidget(self.port_combo)

        self.refresh_button = QPushButton("🔄 Refresh")
        self.refresh_button.setToolTip("Refresh COM ports")
        self.refresh_button.setFixedWidth(100)
        self.refresh_button.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_button)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("9600")
        self.baud_combo.setFont(QFont("Arial", 10))
        self.baud_combo.setFixedWidth(100)
        port_layout.addWidget(QLabel("Baud:"))
        port_layout.addWidget(self.baud_combo)

        port_layout.addStretch()
        main_layout.addLayout(port_layout)

        # --- Second Row: Start, Stop, Save buttons ---
        button_layout = QHBoxLayout()
        button_layout.setSpacing(10)

        self.start_button = QPushButton("▶ Start")
        self.start_button.setStyleSheet("""
            background-color: #4CAF50; 
            color: white; 
            font-weight: bold;
            min-width: 100px;
        """)
        self.start_button.clicked.connect(self.start_reading)
        button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("■ Stop")
        self.stop_button.setStyleSheet("""
            background-color: #F44336; 
            color: white; 
            font-weight: bold;
            min-width: 100px;
        """)
        self.stop_button.clicked.connect(self.stop_reading)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)

        self.save_button = QPushButton("💾 Save CSV")
        self.save_button.setStyleSheet("""
            background-color: #2196F3; 
            color: white;
            min-width: 120px;
        """)
        self.save_button.clicked.connect(self.save_csv)
        button_layout.addWidget(self.save_button)

        self.clear_button = QPushButton("🗑️ Clear Plot")
        self.clear_button.setStyleSheet("""
            background-color: #FF9800; 
            color: white;
        """)
        self.clear_button.clicked.connect(self.clear_plot)
        button_layout.addWidget(self.clear_button)

        button_layout.addStretch()
        main_layout.addLayout(button_layout)

        # --- Third Row: Settings ---
        settings_layout = QHBoxLayout()
        settings_layout.setSpacing(15)
        
        self.checkbox_conf = QCheckBox("Ignore low confidence (0)")
        self.checkbox_conf.setFont(QFont("Arial", 10))
        settings_layout.addWidget(self.checkbox_conf)
        
        self.smooth_checkbox = QCheckBox("Enable Smoothing")
        self.smooth_checkbox.setFont(QFont("Arial", 10))
        self.smooth_checkbox.setChecked(True)
        settings_layout.addWidget(self.smooth_checkbox)
        
        self.max_points_label = QLabel("Max Points:")
        self.max_points_label.setFont(QFont("Arial", 10))
        settings_layout.addWidget(self.max_points_label)
        
        self.max_points_combo = QComboBox()
        self.max_points_combo.addItems(["100", "200", "500", "1000", "2000"])
        self.max_points_combo.setCurrentText("500")
        self.max_points_combo.setFont(QFont("Arial", 10))
        self.max_points_combo.setFixedWidth(80)
        self.max_points_combo.currentTextChanged.connect(self.update_max_points)
        settings_layout.addWidget(self.max_points_combo)
        
        settings_layout.addStretch()
        main_layout.addLayout(settings_layout)

        # --- Plot area ---
        self.figure, self.ax = plt.subplots(figsize=(10, 6))
        self.figure.set_facecolor("#1e1e2f")
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background-color: transparent;")
        main_layout.addWidget(self.canvas, stretch=1)

        # Configure plot appearance (DO NOT clear here)
        self.configure_plot()

        # --- Distance and speed display below plot ---
        display_layout = QHBoxLayout()
        
        # Distance display
        self.distance_label = QLabel("Distance: -- m")
        self.distance_label.setFont(QFont("Arial", 24, QFont.Bold))
        self.distance_label.setAlignment(Qt.AlignCenter)
        self.distance_label.setStyleSheet("""
            color: #00ffff; 
            margin: 5px; 
            background-color: #202030;
            border-radius: 5px;
            padding: 10px;
            min-width: 250px;
        """)
        display_layout.addWidget(self.distance_label)
        
        # Speed displays
        speed_layout = QVBoxLayout()
        
        # Instantaneous speed
        self.instant_speed_label = QLabel("Instant Speed: -- m/min")
        self.instant_speed_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.instant_speed_label.setAlignment(Qt.AlignCenter)
        self.instant_speed_label.setStyleSheet("""
            color: #ff9900; 
            margin: 5px; 
            background-color: #202030;
            border-radius: 5px;
            padding: 5px;
        """)
        speed_layout.addWidget(self.instant_speed_label)
        
        # Average speed
        self.avg_speed_label = QLabel("Avg Speed: -- m/min")
        self.avg_speed_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.avg_speed_label.setAlignment(Qt.AlignCenter)
        self.avg_speed_label.setStyleSheet("""
            color: #ff9900; 
            margin: 5px; 
            background-color: #202030;
            border-radius: 5px;
            padding: 5px;
        """)
        speed_layout.addWidget(self.avg_speed_label)
        
        display_layout.addLayout(speed_layout)
        main_layout.addLayout(display_layout)

        # --- Status bar ---
        self.status_bar = QStatusBar()
        self.status_bar.setFont(QFont("Arial", 9))
        self.status_bar.setStyleSheet("""
            QStatusBar {
                background-color: #202030;
                color: #aaaaaa;
                border-top: 1px solid #444;
            }
        """)
        self.status_bar.showMessage("Ready")
        main_layout.addWidget(self.status_bar)

        self.setLayout(main_layout)

    def configure_plot(self):
        # DO NOT clear the axes here; clearing is done in update_plot
        self.ax.set_facecolor("#202030")
        
        self.ax.set_title("Distance Measurement Over Time", color="white", fontsize=14, pad=20)
        self.ax.set_xlabel("Time (s)", color="white", fontsize=12)
        self.ax.set_ylabel("Distance (m)", color="white", fontsize=12)
        
        self.ax.tick_params(axis='both', colors='white', labelsize=10)
        self.ax.xaxis.set_minor_locator(AutoMinorLocator())
        self.ax.yaxis.set_minor_locator(AutoMinorLocator())
        
        self.ax.grid(True, color='#444', linestyle='--', linewidth=0.5)
        self.ax.grid(True, which='minor', color='#333', linestyle=':', linewidth=0.5)
        
        # Removed legend from here to avoid warning
        
        self.canvas.draw()

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(port.device)
        self.status_bar.showMessage(f"Found {len(ports)} COM ports" if ports else "No COM ports found")

    def start_reading(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        
        if port:
            try:
                self.serial = serial.Serial(port, baud, timeout=1)
                self.serial.flushInput()
                self.timer.start(self.sampling_rate)
                self.start_time = datetime.datetime.now()
                self.log_data = []
                self.data.clear()
                self.time.clear()
                self.speed_calculator = AltimeterSpeed(polydegree=3, listlen=40, tau=2)  # Reset speed calculator
                self.speed_history.clear()
                self.start_button.setEnabled(False)
                self.stop_button.setEnabled(True)
                self.status_bar.showMessage(f"Reading from {port} at {baud} baud")
                print(f"Started reading from {port} at {baud} baud")
            except Exception as e:
                self.status_bar.showMessage(f"Error: {str(e)}")
                print(f"Error opening serial port: {e}")

    def stop_reading(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception as e:
            self.status_bar.showMessage(f"Stop error: {e}")
            print(f"Error closing serial port: {e}")
        self.timer.stop()
        self.status_bar.showMessage("Stopped")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        print("Stopped reading.")

    def save_csv(self):
        if not self.log_data:
            self.status_bar.showMessage("No data to save")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, 
            "Save CSV", 
            f"altimeter_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", 
            "CSV Files (*.csv)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp (s)", "Distance (m)", "Confidence"])
                    writer.writerows(self.log_data)
                self.status_bar.showMessage(f"Data saved to {file_path}")
                print(f"Saved data to {file_path}")
            except Exception as e:
                self.status_bar.showMessage(f"Save error: {e}")
                print(f"Error saving file: {e}")

    def clear_plot(self):
        self.data.clear()
        self.time.clear()
        self.log_data = []
        self.last_value = None
        self.speed_calculator = AltimeterSpeed(polydegree=3, listlen=40, tau=2)  # Reset speed calculator
        self.speed_history.clear()
        self.distance_label.setText("Distance: -- m")
        self.instant_speed_label.setText("Instant Speed: -- m/min")
        self.avg_speed_label.setText("Avg Speed: -- m/min")
        self.configure_plot()
        self.status_bar.showMessage("Plot cleared")

    def update_max_points(self):
        max_points = int(self.max_points_combo.currentText())
        self.data = deque(self.data, maxlen=max_points)
        self.time = deque(self.time, maxlen=max_points)
        self.status_bar.showMessage(f"Max points set to {max_points}")

    def smooth_data(self, y, window_size=5):
        """Apply simple moving average smoothing"""
        if len(y) < window_size:
            return y
        window = np.ones(window_size) / window_size
        return np.convolve(y, window, 'valid')

    def update_plot(self):
        if self.serial and self.serial.in_waiting:
            try:
                raw = self.serial.readline()
                line = raw.decode(errors='ignore').strip()
                print(f"[DEBUG] {line}")

                if line.startswith("#U0") and "," in line:
                    clean = line[4:]
                    parts = clean.split(",")
                    if len(parts) == 2:
                        value_mm = int(parts[0])
                        conf = int(parts[1])

                        if self.checkbox_conf.isChecked() and conf == 0:
                            return

                        value_m = value_mm / 1000.0  # convert mm to meters
                        self.last_value = value_m
                        self.distance_label.setText(f"Distance: {value_m:.3f} m")

                        timestamp = (datetime.datetime.now() - self.start_time).total_seconds()
                        self.time.append(timestamp)
                        self.data.append(value_m)
                        self.log_data.append([timestamp, value_m, conf])
                        
                        # Update speed calculations
                        self.speed_calculator.set_time_depth(timestamp, value_m)
                        instant_speed, avg_speed = self.speed_calculator.get_speeds()
                        
                        # Convert from m/s to m/min for display
                        instant_speed_mmin = instant_speed * 60
                        avg_speed_mmin = avg_speed * 60
                        
                        # Update speed displays
                        self.instant_speed_label.setText(f"Instant Speed: {instant_speed_mmin:.1f} m/min")
                        self.avg_speed_label.setText(f"Avg Speed: {avg_speed_mmin:.1f} m/min")
                        
                        # Store speed for potential plotting
                        self.speed_history.append((timestamp, instant_speed_mmin, avg_speed_mmin))

                        time_array = np.array(self.time)
                        data_array = np.array(self.data)
                        
                        if self.smooth_checkbox.isChecked() and len(data_array) > 5:
                            smoothed_data = self.smooth_data(data_array)
                            plot_time = time_array[len(time_array)-len(smoothed_data):]
                            plot_data = smoothed_data
                        else:
                            plot_time = time_array
                            plot_data = data_array

                        self.ax.clear()

                        self.ax.plot(plot_time, plot_data, 
                                     label=f"Distance (m)",  
                                     color="#00ffff",
                                     linewidth=2,
                                     marker='o',
                                     markersize=4,
                                     markerfacecolor="#ff9900",
                                     markeredgecolor="#ff9900")

                        self.configure_plot()

                        # Add legend here only after plotting
                        self.ax.legend(loc="upper right", 
                                       facecolor="#202030", 
                                       edgecolor="white",
                                       fontsize=10,
                                       labelcolor='white')

                        if len(plot_time) > 0:
                            self.ax.relim()
                            self.ax.autoscale_view()

                        self.canvas.draw()

                        self.status_bar.showMessage(
                            f"Time: {timestamp:.1f}s | Distance: {value_m:.3f} m | Confidence: {conf} | Points: {len(self.data)}"
                        )

            except Exception as e:
                self.status_bar.showMessage(f"Error: {str(e)}")
                print(f"Error processing data: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Set application style for better look
    app.setStyle('Fusion')
    
    window = SerialPlotApp()
    window.show()
    sys.exit(app.exec_())