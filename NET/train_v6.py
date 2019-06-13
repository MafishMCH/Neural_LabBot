# Training for Neural Network - Robot collision
# Author: Antonio Garcia
# This program creates a Neural Network and trains it for deciding the direction of the robot that avoids obstacles
# The Neural network takes the gray-scale camera image as an input and outputs the direction angle


import os
import tensorflow as tf
from tensorflow.core.protobuf import saver_pb2
import scipy.misc
import random
import os
from os import listdir
from os.path import isfile, join
import cv2
import matplotlib.pyplot as plt
import numpy as np
import datetime

# Importing the Keras libraries and packages
import tensorflow as tf
from tensorflow.keras import datasets, layers, models
from keras.models import Sequential
from keras.layers import Conv2D
from keras.layers import MaxPooling2D
from keras.layers import Flatten
from keras.layers import Dense
from tensorflow.keras.callbacks import TensorBoard


""" Training Parameters """
# Image dimensions
img_width = 128
img_height = 96

# Training parameters
training_percentage = 0.75
EPOCHS = 50
batch_size = 32
patience = 10

"""Load data"""
# Vector for training data input
training_data =[]
# Vector for training data output
labels = []

# Path of the image folder
path = "image_dataset/"
# files = [f for f in listdir(path) if isfile(join(path, f))]
# Load all files in directory path and subdirectories
files = []
for r, d, f in os.walk(path):
    for file in f:
        if '.jpg' in file:
            files.append(os.path.join(r, file))

for f in files:
    print(f)

print(len(files))

# Randomize the image order
random.shuffle(files)
# Load images, resize and store direction
i = 1
for file in files:
    print(str(i) + ' ' + file)
    i = i + 1
    direction = file.split("-")[1]
    angle = 0.0
    if direction == "forward":
        angle = 0.0
    elif direction == "left":
        angle = -1.0
    elif direction == "right":
        angle = 1.0
    img = cv2.resize(cv2.imread(file, cv2.IMREAD_GRAYSCALE), (img_width, img_height)).flatten()
    training_data.append(img)
    labels.append(angle)

# Normalize data
training_data = np.array(training_data, dtype="float")/255
labels = np.array(labels)

# Split the data into training data and validation data
x_train = training_data[:int(len(training_data)*training_percentage)]
y_train = labels[:int(len(labels)*training_percentage)]
x_test = training_data[int(len(training_data)*training_percentage):]
y_test = labels[int(len(labels)*training_percentage):]

# Reshape the data for applying 2D convolutions
x_train = x_train.reshape(len(x_train), img_height, img_width, 1)
x_test = x_test.reshape(len(x_test), img_height, img_width, 1)

"""Create Neural Network"""
# Create model for the Neural Network
model = tf.keras.models.Sequential([
    tf.keras.layers.Conv2D(4, (3, 3), padding="same", activation="relu", input_shape=(img_height, img_width, 1)),
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),
    tf.keras.layers.Conv2D(8, (3, 3), padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),
    tf.keras.layers.Conv2D(16, (3, 3), padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),
    tf.keras.layers.Conv2D(32, (3, 3), padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),
    tf.keras.layers.Conv2D(64, (3, 3), padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(32, activation="relu"),
    tf.keras.layers.Dense(16, activation="sigmoid"),

    tf.keras.layers.Dense(1)
])

# Compile neural network
model.compile(loss="mean_squared_error", optimizer='adam', metrics=["accuracy"])


"""Display Neural Network"""
# Create log for the output of the training for visualization in tensorboard
# For running tensorboard type in command line "tensorboard --logdir logs/fit"
# For visualizing tensorboard open the web navigator and type the address
# given after the tensorboard command
log_dir="logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

# Early stopping of the training when the validation loss gets worst
# Note that the patience can be adjusted!
es = tf.keras.callbacks.EarlyStopping(monitor='val_loss', mode='min', patience = patience)


"""Train Neural Network"""
# Train the model
model.fit(x = x_train,
          y = y_train,
          epochs = EPOCHS,
          batch_size = batch_size,
          validation_data = (x_test, y_test),
          callbacks=[tensorboard_callback, es])

# Display model information: number of parameters and layers
model.summary()


"""Save Neural Network"""
model.save('model.h5')
model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
