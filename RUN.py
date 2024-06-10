import time
import math
import board
import busio
import RPi.GPIO as GPIO
import serial
from adafruit_bno055 import BNO055_I2C
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import pynmea2
import numpy as np
import socket
import csv
from threading import Thread

# Set GPIO pin numbering mode
GPIO.setmode(GPIO.BCM)

# Constants
EARTH_RADIUS = 6371.0

running = False

def receive_waypoints():
    waypoints = []

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', 12345))
        s.listen(1)

        print('Waiting for waypoints from laptop...')
        conn, addr = s.accept()

        with conn:
            print(f'Connected to {addr}')
            waypoints_str = conn.recv(1024).decode('utf-8')
            waypoints = eval(waypoints_str)
    print('Waypoint: ' ,waypoints)
    return waypoints

def receive_starting_point():
    starting_point = None

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', 12346))
        s.listen(1)

        print('Waiting for starting point from laptop...')
        conn, addr = s.accept()

        with conn:
            print(f'Connected to {addr}')
            starting_point_str = conn.recv(1024).decode('utf-8')
            starting_point = eval(starting_point_str)
    print('Starting point:', starting_point)
    return starting_point

def control_listener():
    global running

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', 12347))
        s.listen(1)

        while True:
            conn, addr = s.accept()
            with conn:
                command = conn.recv(1024).decode('utf-8')
                if command == 's':
                    running = True
                elif command == 'q':
                    running = False
                    break

# Haversine distance measurement
def haversine_distance(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (math.sin(dlat / 2) ** 2) + (math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * (math.sin(dlon / 2) ** 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c * 1000
    return distance  # Return distance in meters

# Course Angle Estimation
def calculate_bearing(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    dlon = math.radians(lon2 - lon1)

    y = math.sin(dlon) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

# Cross-track error calculation
def cross_track_error(current_position, starting_position, target_position):
    d_current = haversine_distance(starting_position, current_position)
    bearing_current = calculate_bearing(starting_position, current_position)
    bearing_target = calculate_bearing(starting_position, target_position)
    delta_bearing = bearing_target - bearing_current

    xte = math.asin(math.sin(d_current / EARTH_RADIUS) * math.sin(math.radians(delta_bearing))) * EARTH_RADIUS
    return xte * 1000  # Return cross-track error in meters

# Corrected heading calculation
def calculate_corrected_heading(target_bearing, xte, kp=1.0):
    angle_correction = kp * xte
    corrected_bearing = target_bearing + angle_correction
    return (corrected_bearing + 360) % 360

# Calibration routine for BNO055
def auto_calibrate(imu, duration=180):
    start_time = time.time()
    print("Starting automatic calibration of BNO055...")

    while time.time() - start_time < duration:
        sys, gyro, accel, mag = imu.calibration_status
        print(f"Calibration status: sys={sys}, gyro={gyro}, accel={accel}, mag={mag}")

        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("Calibration complete.")
            break
        else:
            time.sleep(1)

    print("Automatic calibration finished.")

# Extended Kalman Filter
class RoverEKF(ExtendedKalmanFilter):
    def __init__(self):
        super().__init__(dim_x=4, dim_z=2)

    def predict(self, dt):
        self.F = np.array([[1, dt, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, dt],
                           [0, 0, 0, 1]])
        self.predict_update()

    def update(self, z):
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
        self.update(z)

# PID control for Deviation Compensation
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

# Motor control function
def motor_control(adjustment, base_speed=30):
    left_speed = base_speed - adjustment
    right_speed = base_speed + adjustment
    
    motor_driver.set_speeds(left_speed, right_speed)

# L298N motor driver class
class L298N:
    def __init__(self, int1, int2, int3, int4, ena, enb):
        self.int1 = int1
        self.int2 = int2
        self.int3 = int3
        self.int4 = int4
        self.ena = ena
        self.enb = enb

        GPIO.setup(self.int1, GPIO.OUT)
        GPIO.setup(self.int2, GPIO.OUT)
        GPIO.setup(self.int3, GPIO.OUT)
        GPIO.setup(self.int4, GPIO.OUT)
        GPIO.setup(self.ena, GPIO.OUT)
        GPIO.setup(self.enb, GPIO.OUT)

        self.pwmA = GPIO.PWM(self.ena, 100)
        self.pwmB = GPIO.PWM(self.enb, 100)

        self.pwmA.start(0)
        self.pwmB.start(0)

    def __del__(self):
        self.stop()
        self.pwmA.stop()
        self.pwmB.stop()
        GPIO.cleanup()

    def set_speeds(self, left_speed, right_speed):
        left_speed = max(min(left_speed, 100), 0)
        right_speed = max(min(right_speed, 100), 0)

        if left_speed > 0:
            GPIO.output(self.int1, GPIO.LOW)
            GPIO.output(self.int2, GPIO.HIGH)
        else:
            GPIO.output(self.int1, GPIO.LOW)
            GPIO.output(self.int2, GPIO.LOW)

        if right_speed > 0:
            GPIO.output(self.int3, GPIO.HIGH)
            GPIO.output(self.int4, GPIO.LOW)
        else:
            GPIO.output(self.int3, GPIO.LOW)
            GPIO.output(self.int4, GPIO.LOW)
        
        self.pwmA.ChangeDutyCycle(left_speed)
        self.pwmB.ChangeDutyCycle(right_speed)

    def stop(self):
        GPIO.output(self.int1, GPIO.LOW)
        GPIO.output(self.int2, GPIO.LOW)
        GPIO.output(self.int3, GPIO.LOW)
        GPIO.output(self.int4, GPIO.LOW)

# Initialize sensors and motor control
i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO055_I2C(i2c)
motor_driver = L298N(int1=6, int2=13, int3=19, int4=26, ena=20, enb=21)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Calibrate BNO055 before navigation
auto_calibrate(imu)
initial_heading = imu.euler[0]

# Waypoints
waypoints = receive_waypoints()

# Starting point
starting_position = receive_starting_point()

waypoint_index = 0
target_position = waypoints[waypoint_index]

control_thread = Thread(target=control_listener)
control_thread.start()

# Initialize the EKF
ekf = RoverEKF()
ekf.x = np.array([0, 0, 0, 0])
ekf.F = np.eye(4)
ekf.H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
ekf.R = np.array([[0.01, 0],
                  [0, 0.01]])
ekf.Q = Q_discrete_white_noise(dim=4, dt=0.1, var=0.1)

# Initialize PID controller
pid_controller = PIDController(kp=1.5, ki=0.0, kd=0.5)

# Create a new CSV file or open an existing one for appending data
csv_file_name = 'position_data.csv'
fieldnames = ['raw_lat', 'raw_lon', 'fused_lat', 'fused_lon']

with open(csv_file_name, mode='w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

while True:
    if running:
    
        dt = 0.2
        
        # Read data from IMU
        imu_data = imu.euler[0]

        if imu_data is None:
            print("Error: Invalid IMU data.")
            continue

        # Read data from GNSS
        gnss_data = ser.readline()
        if gnss_data.startswith(b'$GPGGA'):
            try:
                msg = pynmea2.parse(gnss_data.decode('utf-8'))
                raw_position = np.array([msg.latitude, msg.longitude])
            except pynmea2.ParseError:
                continue

            # Use the EKF to fuse the IMU and GNSS data
            ekf.predict(dt=0.1)
            ekf.update(raw_position)

            # Use the fused position for navigation
            fused_position = ekf.x[0], ekf.x[2]

            # Use the position for navigation
            current_heading = imu_data - initial_heading
            current_position = fused_position
            
            # Append the raw and fused position data to the CSV file
            with open(csv_file_name, mode='a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow({
                    'raw_lat': raw_position[0],
                    'raw_lon': raw_position[1],
                    # 'fused_lat': fused_position[0],
                    # 'fused_lon': fused_position[1],
                })
            
            # Calculate distance to the target
            target_distance = haversine_distance(current_position, target_position)
            target_bearing = calculate_bearing(starting_position, target_position)
            
            # Print GNSS status, rover heading, and distance to the waypoint
            print(f"GNSS position: {current_position}")
            print(f"Rover heading: {current_heading:.2f}°")
            print(f"Distance to waypoint: {target_distance:.2f} meters")
            
            # Calculate cross-track error and corrected heading
            xte = cross_track_error(current_position, starting_position, target_position)
            corrected_heading = calculate_corrected_heading(target_bearing, xte)
            
            print(f"Error from desired course: {xte:.2f} meters")
            
            # Calculate deviation from the desired course
            deviation = corrected_heading - current_heading
            if deviation > 180:
                deviation -= 360
            elif deviation < -180:
                deviation += 360

            print(f"Deviation: {deviation:.2f}°")
            
            # Calculate motor speed adjustment using PID controller
            adjustment = pid_controller.update(deviation, dt)

            # Move the rover
            motor_control(adjustment)

            # Check if the target waypoint is reached
            if target_distance < 0.1:  # 0.1 meter tolerance
                # Move to the next waypoint or stop if all waypoints are reached
                waypoint_index += 1
                if waypoint_index < len(waypoints):
                    starting_position = current_position
                    target_position = waypoints[waypoint_index]
                else:
                    print("All waypoints reached")
                    motor_driver.stop()
                    break
    else:
        motor_driver.stop()
        time.sleep(0.1)
