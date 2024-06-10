from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout
from PyQt5.QtGui import QPixmap
import socket
import os
from PyQt5.QtWidgets import QApplication

# Set the environment variables for scaling behavior
os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"

API_KEY = "AIzaSyBWhJ_T-rE0R8vt6i6EWh0Gz3cx533M2so"

def send_waypoints():
    try:
        latitudes = latitudes_edit.text().split(',')
        longitudes = longitudes_edit.text().split(',')

        if len(latitudes) != len(longitudes):
            raise ValueError('Number of latitudes and longitudes must be equal')

        waypoints = [(float(lat.strip()), float(lon.strip())) for lat, lon in zip(latitudes, longitudes)]

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('raspberrypi.local', 12345))
            s.sendall(str(waypoints).encode('utf-8'))

        status_label.setText('Waypoints sent successfully!')

    except Exception as e:
        status_label.setText(f'Failed to send waypoints: {e}')

def send_starting_point():
    try:
        start_lat = float(start_latitude_edit.text())
        start_lon = float(start_longitude_edit.text())

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('raspberrypi.local', 12346))
            s.sendall(f"{start_lat},{start_lon}".encode('utf-8'))

        status_label.setText('Starting point sent successfully!')

    except Exception as e:
        status_label.setText(f'Failed to send starting point: {e}')

def start_rover():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('raspberrypi.local', 12347))
            s.sendall(b's')
        status_label.setText('Rover started.')
    except Exception as e:
        status_label.setText(f'Failed to start rover: {e}')

def stop_rover():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('raspberrypi.local', 12347))
            s.sendall(b'q')
        status_label.setText('Rover stopped.')
    except Exception as e:
        status_label.setText(f'Failed to stop rover: {e}')

app = QApplication([])

window = QWidget()
window.setWindowTitle('Waypoint Sender')

main_layout = QHBoxLayout(window)
control_layout = QVBoxLayout()
main_layout.addLayout(control_layout)

latitudes_label = QLabel('Enter latitudes (separated by commas):')
control_layout.addWidget(latitudes_label)

latitudes_edit = QLineEdit()
control_layout.addWidget(latitudes_edit)

longitudes_label = QLabel('Enter longitudes (separated by commas):')
control_layout.addWidget(longitudes_label)

longitudes_edit = QLineEdit()
control_layout.addWidget(longitudes_edit)

send_button = QPushButton('Send Waypoints')
send_button.clicked.connect(send_waypoints)
control_layout.addWidget(send_button)

status_label = QLabel('')
control_layout.addWidget(status_label)

control_layout.addWidget(QLabel('Enter starting latitude:'))
start_latitude_edit = QLineEdit()
control_layout.addWidget(start_latitude_edit)

control_layout.addWidget(QLabel('Enter starting longitude:'))
start_longitude_edit = QLineEdit()
control_layout.addWidget(start_longitude_edit)

send_starting_point_button = QPushButton('Send Starting Point')
send_starting_point_button.clicked.connect(send_starting_point)
control_layout.addWidget(send_starting_point_button)

status_label = QLabel('')
control_layout.addWidget(status_label)

start_button = QPushButton("START")
start_button.clicked.connect(start_rover)

stop_button = QPushButton("STOP")
stop_button.clicked.connect(stop_rover)

control_layout.addWidget(start_button)
control_layout.addWidget(stop_button)

# Add the map image
map_image_label = QLabel()
map_image_pixmap = QPixmap('/Users/VUTRU/Downloads/MAP.png')
map_image_label.setPixmap(map_image_pixmap.scaled(800, 800, aspectRatioMode=True))
main_layout.addWidget(map_image_label)

window.show()
app.exec_()