# -*- coding: utf-8 -*-
"""
Detection of arrows for robot control using CNN and ROS

This script creates, trains, evaluates, and saves a CNN model to recognize the direction of hand-drawn arrows (up, down, left, right).

@author: Uriel Martinez-Hernandez (template creator)

"""

# Load required packages
import scipy.io as sio
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential # type: ignore
from tensorflow.keras.models import load_model
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras.optimizers import SGD
import matplotlib.pyplot as plt
from PIL import Image
import glob
import os

os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

# List of arrow classes
namesList = ['up', 'down', 'left', 'right']

# Folder names of train and testing images
imageFolderPath = r'Database_arrows'
imageFolderTrainingPath = imageFolderPath + r'/train'
imageFolderTestingPath = imageFolderPath + r'/validation'
imageTrainingPath = []
imageTestingPath = []

# Obtain all images from the training and testing folders
for i in range(len(namesList)):
    trainingLoad = imageFolderTrainingPath + '/' + namesList[i] + '/*.jpg'
    testingLoad = imageFolderTestingPath + '/' + namesList[i] + '/*.jpg'
    imageTrainingPath = imageTrainingPath + glob.glob(trainingLoad)
    imageTestingPath = imageTestingPath + glob.glob(testingLoad)

# Print number of training and testing images
print(f"Number of training images: {len(imageTrainingPath)}")
print(f"Number of testing images: {len(imageTestingPath)}")

# Resize images to speed up training process
updateImageSize = [128, 128]
tempImg = Image.open(imageTrainingPath[0]).convert('L')
tempImg.thumbnail(updateImageSize, Image.ANTIALIAS)
[imWidth, imHeight] = tempImg.size

# Create space to load training and testing images
x_train = np.zeros((len(imageTrainingPath), imHeight, imWidth, 1))
x_test = np.zeros((len(imageTestingPath), imHeight, imWidth, 1))

# Load training images
for i in range(len(x_train)):
    tempImg = Image.open(imageTrainingPath[i]).convert('L')
    tempImg.thumbnail(updateImageSize, Image.ANTIALIAS)
    x_train[i, :, :, 0] = np.array(tempImg, 'f')

# Load testing images
for i in range(len(x_test)):
    tempImg = Image.open(imageTestingPath[i]).convert('L')
    tempImg.thumbnail(updateImageSize, Image.ANTIALIAS)
    x_test[i, :, :, 0] = np.array(tempImg, 'f')

# Create space to load training and testing labels
y_train = np.zeros((len(x_train),))
y_test = np.zeros((len(x_test),))

# Load training labels
countPos = 0
for i in range(len(namesList)):
    for j in range(round(len(imageTrainingPath) / len(namesList))):
        y_train[countPos] = i
        countPos += 1

# Load testing labels, wouldnt work if the number of images is not the same for each class
countPos = 0
for i in range(len(namesList)):
    for j in range(round(len(imageTestingPath) / len(namesList))):
        y_test[countPos] = i
        countPos += 1

# Convert labels to one-hot format
y_train = tf.keras.utils.to_categorical(y_train, len(namesList))
y_test = tf.keras.utils.to_categorical(y_test, len(namesList))

# Normalize image data
x_train /= 255.0
x_test /= 255.0

# Create CNN model
#model = Sequential([
#    Conv2D(32, (3, 3), activation='relu', input_shape=(imHeight, imWidth, 1)),
#    MaxPooling2D((2, 2)),
#    Conv2D(64, (3, 3), activation='relu'),
#    MaxPooling2D((2, 2)),
#    Flatten(),
#    Dense(128, activation='relu'),
#    Dropout(0.5),
#    Dense(len(namesList), activation='softmax')
#])

model = Sequential([
    Conv2D(32, (3, 3), activation='relu', input_shape=(imHeight, imWidth, 1)),
    MaxPooling2D((2, 2)),
    Dropout(0.25),
    Flatten(),
    Dense(128, activation='relu'),
    Dropout(0.5),
    Dense(4, activation='softmax')
])

# Compile the model
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

# Train the model
history = model.fit(x_train, y_train, epochs=5, batch_size=32, validation_split=0.2)

# Evaluate the model
loss, accuracy = model.evaluate(x_test, y_test)
print(f"Test accuracy: {accuracy * 100:.2f}%")

# Save the model
model_path = "CNN_Model.keras"
model.save(model_path)


# Plot accuracy and loss
plt.figure(figsize=(12, 5))

# Accuracy plot
plt.subplot(1, 2, 1)
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.title('Model Accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

# Loss plot
plt.subplot(1, 2, 2)
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

plt.show()

print('OK')
