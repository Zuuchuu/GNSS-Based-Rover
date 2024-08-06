import numpy as np
import os
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Dense, Activation, Dropout, Flatten, Input, Reshape, Conv2D, MaxPooling2D
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.utils import to_categorical
import cv2
from sklearn.model_selection import train_test_split
path = "D:/AI/Train"

batchsize = 50
steps_per_epoch = 2000
epochs = 30
imageDimension = (32, 32, 3)
test_ratio = 0.2
valid_ratio = 0.2

classes = {0: 'Speed limit (20km/h)',
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

def load_data(data_dir):
    images = []
    labels = []
    data_path = os.path.join(data_dir)
    number_of_labels = len(os.listdir(data_path))
    for sub in range(number_of_labels):
        sub_folder = os.path.join(data_path, str(sub))
        images_in_subfolder = []
        for image in os.listdir(sub_folder):
            images_in_subfolder.append(image)
        for image in images_in_subfolder:
            image_path = os.path.join(data_path, str(sub), image)
            labels.append(sub)
            img = cv2.imread(image_path)
            res = cv2.resize(img, (32, 32), interpolation=cv2.INTER_AREA)
            images.append(res)
    return (np.array(images), np.array(labels))

def preprocessing(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.equalizeHist(img)
    img = img / 255
    return img

def get_model():
    model = Sequential([ 
        Conv2D(40, (3, 3), activation='relu',input_shape=imageDimension),
        MaxPooling2D((2, 2)),
        Conv2D(40, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(128, activation='relu'),
        Dense(128, activation='relu'),
        Dense(128, activation='relu'),
        Dropout(0.2),
        Dense(128, activation='relu'),
        Dense(43, activation='softmax')
    ])   
    model.compile(
        optimizer="adam",
        loss="categorical_crossentropy",
        metrics=["accuracy"]
    )

    return model

images, labels = load_data(path)
labels = to_categorical(labels)
x_train, x_test, y_train, y_test = train_test_split(
        np.array(images), np.array(labels), test_size=0.2 ) 
model = get_model()
history = model.fit(x_train, y_train, epochs=epochs)
loss = history.history["loss"]
accuracy = history.history["accuracy"]
model.evaluate(x_test, y_test, verbose=2)

model.save('D:/AI/trained_model.h5')


