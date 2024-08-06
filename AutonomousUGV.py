import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import numpy as np
import time
import os
import matplotlib.pyplot as plt
from keras.models import Sequential, Model, load_model
import threading
import RPi.GPIO as GPIO
import time 
import smbus
import serial
import pynmea2
import math
import sys
import busio
from adafruit_bno055 import BNO055_I2C
import json
import board
import datetime

port='/dev/ttyTHS1'
baudrate=9600
HEADING_DIFF = []
list_label = [""]
data = []
GPS = [""]


class_traffic = {0: 'Speed limit (20km/h)',
           1: 'Speed limit (30km/h)', 
           2: 'Speed limit (50km/h)', 
           3: 'Speed limit (60km/h)', 
           4: 'Speed limit (70km/h)', 
           5: 'Speed limit (80km/h)', 
           6: 'End of speed limit (80km/h)', 
           7: 'Speed limit (100km/h)', 
           8: 'Speed limit (120km/h)', 
           9: 'No passing', 
           10: 'No passing veh over 3.5 tons', 
           11: 'Right-of-way at intersection', 
           12: 'Priority road', 
           13: 'Yield', 
           14: 'Stop',
           15: 'No vehicles', 
           16: 'Veh > 3.5 tons prohibited', 
           17: 'No entry', 
           18: 'General caution', 
           19: 'Dangerous curve left', 
           20: 'Dangerous curve right', 
           21: 'Double curve', 
           22: 'Bumpy road', 
           23: 'Slippery road', 
           24: 'Road narrows on the right', 
           25: 'Road work', 
           26: 'Traffic signals', 
           27: 'Pedestrians', 
           28: 'Children crossing', 
           29: 'Bicycles crossing', 
           30: 'Beware of ice/snow',
           31: 'Wild animals crossing', 
           32: 'End speed + passing limits', 
           33: 'Turn right ahead', 
           34: 'Turn left ahead', 
           35: 'Ahead only', 
           36: 'Go straight or right', 
           37: 'Go straight or left', 
           38: 'Keep right', 
           39: 'Keep left', 
           40: 'Roundabout mandatory', 
           41: 'End of no passing', 
           42: 'End no passing veh > 3.5 tons' }
model = load_model("/media/dtvt/5E0C-5CCD/thesis//old_trained_model.h5")
IN1 = 11
IN2 = 12
IN3 = 15
IN4 = 16
ENABLE_A = 32
ENABLE_B = 33

EARTH_RADIUS = 6371

pi = 3.14
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
RIGHT_LINE_SENSOR = 35
LEFT_LINE_SENSOR = 36
def save_to_json():
    with open('vehicle_data.json', 'w') as f:
        json.dump(data, f, indent=4)

def get_gps_coordinates():
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                print(line)
                if line.startswith('$GPRMC'):
                    print("Start reading data")
                    try:
                        gga = pynmea2.parse(line)
                        latitude = gga.latitude
                        longitude = gga.longitude
                        # print("Latitude: ", round(latitude, 5))
                        # print("Longitude: ", round(longitude, 5))
                        # print("Finish reading")
                        coordinate = (latitude,longitude)
                        GPS.append(coordinate)
                        return latitude, longitude
                    except pynmea2.ParseError as e:
                        print(f"Parse error: {e}")
                        continue
                else:
                    print("Could not get value")
            except UnicodeDecodeError as e:
                    print(f"Decode error: {e}")
                    continue

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        return None, None

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

def haversine_distance(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (math.sin(dlat / 2) ** 2) + (math.cos(math.radians(lat1)) * math.cos (math.radians(lat2)) * (math.sin(dlon / 2) ** 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c * 1000 
    return distance
def calculate_bearing(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    dlon = math.radians(lon2 - lon1)

    y = math.sin(dlon) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


class L298N:
    def __init__(self, IN1, IN2, IN3, IN4, ENABLE_A, ENABLE_B):
        self.IN1 = IN1
        self.IN2 = IN2
        self.IN3 = IN3
        self.IN4 = IN4
        self.ENABLE_A = ENABLE_A
        self.ENABLE_B = ENABLE_B

        GPIO.setup(IN1,GPIO.OUT)
        GPIO.setup(IN2,GPIO.OUT)
        GPIO.setup(IN3,GPIO.OUT)
        GPIO.setup(IN4,GPIO.OUT)

        GPIO.setup(LEFT_LINE_SENSOR,GPIO.IN)
        GPIO.setup(RIGHT_LINE_SENSOR,GPIO.IN)

        GPIO.setup(ENABLE_A,GPIO.OUT)
        GPIO.setup(ENABLE_B,GPIO.OUT)
        self.pwm_a = GPIO.PWM(ENABLE_A, 1000)
        self.pwm_b = GPIO.PWM(ENABLE_B, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
    
    def __del__(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

    def move_fw(self,speed1,speed2):
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

        self.pwm_a.ChangeDutyCycle(speed2)
        self.pwm_b.ChangeDutyCycle(speed1)
        

    def turn_left(self,speed1,speed2):
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

        self.pwm_a.ChangeDutyCycle(speed2)
        self.pwm_b.ChangeDutyCycle(speed1)
        print("turn left")
        
    def turn_right(self,speed1,speed2):
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

        self.pwm_a.ChangeDutyCycle(speed2)
        self.pwm_b.ChangeDutyCycle(speed1)
        print("turn right")
        
    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
car = L298N(IN1,IN2,IN3,IN4,ENABLE_A,ENABLE_B)
i2c =busio.I2C(board.SCL_1,board.SDA_1)
sensor = BNO055_I2C(i2c)

def main_window():
    global a
    a = tk.Tk(className=' Car Observation')
    a.geometry("800x600")
    a.configure(background="white")
    a.grid_columnconfigure(0, weight=1)
    a.grid_rowconfigure(0, weight=1)
    
    big_label = tk.Label(a, text="Develop a Traffic Sign Recognition and GPS Positioning System for Autonomous Vehicles", font=("Times New Roman", 16), fg="Black", bg="White")
    big_label1 = tk.Label(a, text="Nguyễn Hoàng Tuấn - 20200403", font=("Times New Roman", 16), fg="Black", bg="White")
    big_label2 = tk.Label(a, text="Ths. Đặng Tấn Phát", font=("Times New Roman", 16), fg="Black", bg="White")
    big_label.grid(row=1, column=0)
    big_label1.grid(row=2, column=0,pady=10)
    big_label2.grid(row=3, column=0,pady=10)
    # Create a frame with 1 column and 2 rows
    frame = tk.Frame(a, bg="white")
    frame.grid()
    for i in range(0,6,1):
        frame.grid_columnconfigure(i, weight=1)
        frame.grid_rowconfigure(i, weight=1)

    background = Image.open("hcmus.jpg")
    background = background.resize((200, 200), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background)
    background_label = tk.Label(frame, image=background_photo, bg="white")
    background_label.grid(row=0, column=0)

    button_start = tk.Button(frame, text='Open System', width=15, command=lambda: [a.destroy(), login_window()], bg="blue", fg="white", font=("Times New Roman", 15, "bold"))
    button_start.grid(row=4, column=0, pady=50)
    def login_window():
        global m
        global angle_label,speed_label,distance_label,predicted_label
        speed = 0
        angle = 0.0
        distance = 0.0

        m = tk.Tk(className='Car Observation')
        m.geometry("800x600")

        login_frame = tk.Frame(m)
        login_frame.grid(row=0, column=0, padx=20, pady=20)
        angle_label = tk.Label(login_frame,text=str(angle) ,font=("Times New Roman", 12, "bold"))
        speed_label = tk.Label(login_frame,text = str(speed) ,font=("Times New Roman", 12, "bold"))
        distance_label = tk.Label(login_frame,text=str(distance) ,font=("Times New Roman", 12, "bold"))
        predicted_label = tk.Label(login_frame, font=("Times New Roman", 12, "bold"))
        

        def car_running():
            auto_calibrate(sensor)        
            lat = float(entry_x.get())
            long = float(entry_y.get())
            intended_coordinate = (lat,long)
            print("Car is running")
            try:
                while list_label:
                    init_coordinate = get_gps_coordinates()
                    if init_coordinate != '':
                        while True:
                            current_time = datetime.datetime.now()
                            date = current_time.date()
                            Time = current_time.time()
                            current_heading = sensor.euler[0]
                            angle_label.config(text=str(current_heading)) 

                            target_bearing = calculate_bearing(init_coordinate,intended_coordinate)
                
                            heading_diff = (target_bearing - current_heading ) % 360
                            HEADING_DIFF.append(heading_diff)
                            
                            if heading_diff > 180:
                                heading_diff -= 360
                            elif heading_diff < -180:
                                heading_diff += 360
                            if heading_diff > 5:
                                car.turn_right(30,30)
                            if heading_diff < -5:
                                car.turn_left(30,30)

                            
                            elif heading_diff < 5 and heading_diff > -5:
                                while True:
                                    current_coordinate = get_gps_coordinates()

                                    final_label = list_label[-1]
                                    predicted_label.config(text=final_label)

                                    distance = haversine_distance(current_coordinate,intended_coordinate)
                                    distance_label.config(text=str(distance))

                                    if final_label == 'Speed limit (20km/h)':
                                        speed = 15
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 20km/h")
                                    elif final_label == 'Speed limit (30km/h)':
                                        speed = 25
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 30km/h")
                                    elif final_label == 'Speed limit (50km/h)':
                                        speed = 45
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 50km/h")
                                    elif final_label == 'Speed limit (60km/h)':
                                        speed = 55
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 60km/h")
                                    elif final_label == 'Speed limit (70km/h)':
                                        speed = 65
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 70km/h")
                                    elif final_label == 'Speed limit (80km/h)':
                                        speed = 75
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 80km/h")
                                    elif final_label == 'Speed limit (100km/h)':
                                        speed = 90
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 100km/h")
                                    elif final_label == 'Speed limit (120km/h)':
                                        speed = 100
                                        speed_label.config(text=str(speed))
                                        print("Speed limit: 120km/h")
                                    elif final_label == 'Stop' or final_label =="person" or final_label =="car" or final_label =="car" or final_label =="bus" or final_label =="truck" or final_label == "train" or final_label == "cat" or final_label == "dog" or final_label =="stop sign":
                                        car.stop()
                                        time.sleep(5)
                                        list_label.clear()
                                        list_label.append("")
                                        
                                    elif final_label == 'Pedestrians':
                                        speed = 20
                                        speed_label.config(text=str(speed))
                                        print("Slow down the speed due to being on Pedestrian Area")
                                    else:
                                        speed = 30
                                        speed_label.config(text=str(speed))
                                        print("Car is running with base speed")
                                    
                                    tree.insert('', 'end', values=(date,Time,intended_coordinate, speed,distance,target_bearing, final_label))
                                    data.append({
                                        "Date: ": date.isoformat(),
                                        "Time: ": Time.isoformat(),
                                        "Intended Coordinate:": intended_coordinate,
                                        "Vehicle Speed: ": speed,
                                        "Distance: ": distance,
                                        "Current coordinate: ": current_coordinate,
                                        "Label: ": final_label
                                    })
                                    save_to_json()
                                    if distance < 0.5: 
                                        car.stop()
                                        time.sleep(5)
                                        break
                                    else:
                                        car.move_fw(speed,speed)
                                        time.sleep(0.1)
                                        
                                    
            except KeyboardInterrupt:
                car.__del__()
        def using_yolov3(model, class_traffic):
            global tracking
            global list_label,cap
            tracking = 0
            net = cv2.dnn.readNet("/media/dtvt/5E0C-5CCD/thesis/yolov3_training_last.weights","/media/dtvt/5E0C-5CCD/thesis/yolov3.cfg") 
            
            classes = []
            with open("/media/dtvt/5E0C-5CCD/thesis/coco.names","r") as f:
                classes = [line.strip() for line in f.readlines()]
            print(classes)
            layer_names = net.getLayerNames()
            outputlayers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
            colors= np.random.uniform(0,255,size=(len(classes),3))
            cap = cv2.VideoCapture(0)
            font = cv2.FONT_HERSHEY_PLAIN
            starting_time= time.time()
            frame_id = 0

            while True:
                _,frame= cap.read() # 
                frame_id+=1
                
                height,width,channels = frame.shape
                blob = cv2.dnn.blobFromImage(frame,0.00392,(64,64),(0,0,0),True,crop=False)   
                net.setInput(blob)
                outs = net.forward(outputlayers)

                class_ids=[]
                confidences=[]
                boxes=[]
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.3:

                            center_x= int(detection[0]*width)
                            center_y= int(detection[1]*height)
                            w = int(detection[2]*width)
                            h = int(detection[3]*height)

                            x=int(center_x - w/2)
                            y=int(center_y - h/2)
                            roi = frame[y:y+h,x:x+w]
                            cv2.imwrite("/media/dtvt/5E0C-5CCD/thesis/traffic_signal.jpg", roi)
                            tracking = 1  
                            boxes.append([x,y,w,h])
                            confidences.append(float(confidence)) 
                            class_ids.append(class_id) 
                indexes = cv2.dnn.NMSBoxes(boxes,confidences,0.2,0.8)


                for i in range(len(boxes)):
                    if i in indexes:
                        x,y,w,h = boxes[i]
                        label = str(classes[class_ids[i]])
                        confidence= confidences[i]
                        color = colors[class_ids[i]]
                        cv2.rectangle(frame,(x,y),(x+w,y+h),color,2)
                        cv2.putText(frame,label+" "+str(round(confidence,2)),(x,y+30),font,2,(0,255,0),2)
                        
                elapsed_time = time.time() - starting_time
                fps=frame_id/elapsed_time
                cv2.putText(frame,"FPS:"+str(round(fps,2)),(10,50),font,2,(0,0,0),1)
                
                # cv2.imshow("Image",frame)
                img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)
                if tracking == 1:
                    global final_label
                    img = cv2.imread("/media/dtvt/5E0C-5CCD/thesis/traffic_signal.jpg")
                    confidence_list = []
                    resized_frame = cv2.resize(img, (30, 30), interpolation=cv2.INTER_AREA)
                    reshaped_frame = resized_frame.reshape(1, 30, 30, 3) 
                    prediction = model.predict(reshaped_frame)
                    predicted_label = np.argmax(prediction)
                    confidence = np.max(prediction)
                    confidence_list.append(confidence)
                    final_label = class_traffic[predicted_label]
                    confidence = np.round(confidence*100,2)
                    print(f"{final_label}: {confidence}%")
                    cv2.putText(img, final_label+" "+str(np.round(confidence,2)), (10, 20 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow("The result",img)
                    list_label.append(final_label)
                    tracking = 0
            
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    tracking = 0
                    cap.release()
                    cv2.destroyAllWindows()
                    car.__del__()
                    break
        def using_yolov3_obstacle(model, class_traffic):
            global tracking
            global list_label,cap
            tracking = 0
            net = cv2.dnn.readNet("/media/dtvt/5E0C-5CCD/thesis/yolov3_obstacle.weights","/media/dtvt/5E0C-5CCD/thesis/yolov3_obstacle.cfg") 
            
            classes = []
            with open("/media/dtvt/5E0C-5CCD/thesis/coco_obstacle.names","r") as f:
                classes = [line.strip() for line in f.readlines()]
            print(classes)
            layer_names = net.getLayerNames()
            outputlayers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
            colors= np.random.uniform(0,255,size=(len(classes),3))
            cap = cv2.VideoCapture(0)
            font = cv2.FONT_HERSHEY_PLAIN
            starting_time= time.time()
            frame_id = 0

            while True:
                _,frame= cap.read() # 
                frame_id+=1
                
                height,width,channels = frame.shape
                blob = cv2.dnn.blobFromImage(frame,0.00392,(64,64),(0,0,0),True,crop=False)   
            
                net.setInput(blob)
                outs = net.forward(outputlayers)

                class_ids=[]
                confidences=[]
                boxes=[]
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.3:

                            center_x= int(detection[0]*width)
                            center_y= int(detection[1]*height)
                            w = int(detection[2]*width)
                            h = int(detection[3]*height)

                            x=int(center_x - w/2)
                            y=int(center_y - h/2)
                            roi = frame[y:y+h,x:x+w]
            
                            # tracking = 1  
                            boxes.append([x,y,w,h])
                            confidences.append(float(confidence)) 
                            class_ids.append(class_id) 
                indexes = cv2.dnn.NMSBoxes(boxes,confidences,0.2,0.8)


                for i in range(len(boxes)):
                    if i in indexes:
                        x,y,w,h = boxes[i]
                        label = str(classes[class_ids[i]])
                        confidence= confidences[i]
                        color = colors[class_ids[i]]
                        cv2.rectangle(frame,(x,y),(x+w,y+h),color,2)
                        cv2.putText(frame,label+" "+str(round(confidence,2)),(x,y+30),font,2,(0,255,0),2)
                        list_label.append(label)
                        
                elapsed_time = time.time() - starting_time
                fps=frame_id/elapsed_time
                cv2.putText(frame,"FPS:"+str(round(fps,2)),(10,50),font,2,(0,0,0),1)
                
                # cv2.imshow("Image",frame)
                
                img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)

        def start_thread():
            lat = entry_x.get()
            long = entry_y.get()
            if lat=="" or long=="":
                messagebox.showerror("Error", "Latitude and Longitude fields cannot be empty.")

            else:
                button_start_camera.config(state="disabled")
                button_start_camera_obstacle.config(state="disabled")
                video_thread = threading.Thread(target = using_yolov3,args=(model,class_traffic))
                car = threading.Thread(target=car_running)
                video_thread.start()
                car.start()

        def start_thread_obstacle():
            lat = entry_x.get()
            long = entry_y.get()
            if lat=="" or long=="":
                messagebox.showerror("Error", "Latitude and Longitude fields cannot be empty.")

            else:
                button_start_camera.config(state="disabled")
                button_start_camera_obstacle.config(state="disabled")
                video_thread = threading.Thread(target = using_yolov3_obstacle,args=(model,class_traffic))
                car = threading.Thread(target=car_running)
                video_thread.start()
                car.start()
                

        def stop():
            sys.exit(0)
        # Configure the login_frame grid
        for i in range(0,9,1):
            login_frame.grid_columnconfigure(i, weight=1)
            login_frame.grid_rowconfigure(i, weight=1)


        label_x = tk.Label(login_frame, text='Intended Latitude', font=("Times New Roman", 12, "bold"))
        entry_x = tk.Entry(login_frame, font=("Times New Roman", 12, "bold"))

        label_y = tk.Label(login_frame, text='Intended Longitude', font=("Times New Roman", 12, "bold"))
        entry_y = tk.Entry(login_frame, font=("Times New Roman", 12, "bold"))

        Label = tk.Label(login_frame, text='Label', font=("Times New Roman", 12, "bold"))
        

        steering_angle = tk.Label(login_frame, text='Heading', font=("Times New Roman", 12, "bold"))
        

        speed = tk.Label(login_frame, text='Speed', font=("Times New Roman", 12, "bold"))
        

        distance = tk.Label(login_frame, text='Distance', font=("Times New Roman", 12, "bold"))
        

        label_x.grid(row=0, column=1, pady=5)
        entry_x.grid(row=0, column=2, pady=5)

        label_y.grid(row=1, column=1, pady=5)
        entry_y.grid(row=1, column=2, pady=5)

        Label.grid(row=2, column=1, pady=5)
        predicted_label.grid(row=2, column=2, pady=5)

        steering_angle.grid(row=3, column=1, pady=5)
        angle_label.grid(row=3, column=2, pady=5)

        speed.grid(row=4, column=1, pady=5)
        speed_label.grid(row=4, column=2, pady=5)

        distance.grid(row=5, column=1, pady=5)
        distance_label.grid(row=5, column=2, pady=5)
        
        button_start_camera = tk.Button(login_frame, text='Traffic Sign Detection', width=15, command=start_thread, bg="green", fg="black", font=("Times New Roman", 12, "bold"))
        button_stop = tk.Button(login_frame, text='Stop', width=15, command=stop, bg="red", fg="black", font=("Times New Roman", 12, "bold"))
        button_start_camera_obstacle = tk.Button(login_frame, text='Obstacle Detection', width=15, command=start_thread_obstacle, bg="yellow", fg="black", font=("Times New Roman", 12, "bold"))

        button_start_camera.grid(row=7, column=1, pady=10)
        button_stop.grid(row=7, column=2, pady=10)
        button_start_camera_obstacle.grid(row=7, column=3, pady=10)

        table_frame = tk.Frame(m)
        table_frame.grid(row=0, column=1, sticky="nsew")
        columns = ("Date","Time","Coordinate","Speed","Distance","Bearing", "Label")
        tree = ttk.Treeview(table_frame, columns=columns, show='headings')
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, anchor="center")
        tree.pack(expand=True, fill='both')

        video_frame = tk.Frame(m)
        video_frame.grid(row=1, column=0, sticky="nsew")
        video_label = tk.Label(video_frame)
        video_label.pack(expand=True, fill='both')

        def close_window():
            if messagebox.askokcancel("Quit","Do you want to quit?"):
                cap.release()
                cv2.destroyAllWindows()
                m.destroy()
                car.__del__()
        m.protocol("WM_DELETE_WINDOW", close_window)
        
        m.mainloop()
        
    def close_window():
        if messagebox.askokcancel("Quit","Do you want to quit?"):
            a.destroy()
    a.protocol("WM_DELETE_WINDOW", close_window)
    a.mainloop()
main_window()
