#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
import os
from sklearn.model_selection import train_test_split
from keras.layers import Dense, Activation, Conv1D, MaxPooling1D, Flatten, Dropout, BatchNormalization
from keras import models
from keras import layers
from keras.callbacks import ModelCheckpoint


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


folderIN = '/home/radhen/Documents/expData/motion3/lr_bf_probeUP/bf'
filename = [f for f in os.listdir(folderIN) if f.endswith('.txt')]
data_roll = {} #each arr in dict a single load un-load curve for a given pitch and roll
data_pitch = {}
data_cycles = {}
for i in range(1): # roll
    for j in range(3): # pitch
        for k in range(5): # load-unload cycles
            data_cycles[k] = np.loadtxt(folderIN + '/lr_bf_{}_{}_{}.txt'.format(i,j,k))
        data_pitch[j] = data_cycles
        data_cycles = {}

data_all = {}
data_all = data_pitch


for i in range(len(data_all)):
    plt.figure()
    for j in range(len(data_all[0])):
        plt.plot(data_all[i][j][100:,14])
plt.show()
print ('wait')

# Filter requirements.
# order = 6
# fs = 40.0       # sample rate, Hz
# cutoff = 2.0  # desired cutoff frequency of the filter, Hz
# b, a = butter_lowpass(cutoff, fs, order)
# data = lr[:,15]
# data_lowpassed = butter_lowpass_filter(data, cutoff, fs, order)

# plt.plot(lr[:,15]/np.power(2,16))
# plt.plot(lr[:,14]/np.power(2,24))

# dydx_ir = np.gradient(lr[:,15],1)
# dydx_ir_norm = np.abs(dydx_ir)/np.max(dydx_ir)
# peaks, _ = find_peaks(dydx_ir_norm,height=0.05)


WS = 100
count = 0
noe = 0

for i in range(len(data_all)):
    for j in range(len(data_all[0])):
        noe += data_all[i][j].shape[0]-WS-1


X = np.zeros((noe, WS, 2))
Y = np.zeros((noe, 3))
for i in range(len(data_all)):
    for j in range(len(data_all[0])):
        # noe += data_all[i][j].shape[0]-WS-1
        for k in range(data_all[i][j].shape[0]-WS-1):
            X[count,:,:] = data_all[i][j][k:WS+k,14:16] # baro, ir
            Y[count, 0] = data_all[i][j][WS+k, 8] # Force
            Y[count, 1] = data_all[i][j][WS+k, 6] # pitch
            Y[count, 2] = data_all[i][j][WS+k, 5] # roll
            count += 1


print(noe)

X[:,:,0] /= np.power(2,24)
X[:,:,1] /= np.power(2,16)

train_x, test_x, train_y, test_y = train_test_split(X,Y,test_size=0.2,random_state=0)

# Start neural network
network = models.Sequential()
network.add(Conv1D(filters=8, kernel_size=4, input_shape=(WS, 2)))
network.add(MaxPooling1D(2))
network.add(Conv1D(filters=32, kernel_size=4))
network.add(MaxPooling1D(4))
network.add(Conv1D(filters=64, kernel_size=4))
network.add(MaxPooling1D(8))
network.add(Flatten())
# network.add(layers.Dense(units=64, activation='relu'))
network.add(layers.Dense(units=42, activation='relu'))
network.add(layers.Dense(units=28, activation='relu'))
network.add(layers.Dense(units=18, activation='relu'))
network.add(layers.Dense(units=12, activation='relu'))
network.add(layers.Dense(units=8, activation='relu'))
network.add(layers.Dense(units=5, activation='relu'))
network.add(layers.Dense(units=3))

network.compile(loss='mse', optimizer='RMSprop', metrics=['mse'])

print (network.summary())

# Checkpoint. Useful link: https://machinelearningmastery.com/check-point-deep-learning-models-keras/
filepath="motion3_weights.best.hdf5"
checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

# Train neural network
history = network.fit(train_x, # Features
                      train_y, # Target vector
                      epochs=50, # Number of epochs
                      verbose=1, # No output
                      batch_size=25, # Number of observations per batch
                      validation_data=(test_x, test_y), # Data for evaluation
                      callbacks=callbacks_list)

loss_and_metrics = network.evaluate(test_x, test_y, batch_size=10)
print (loss_and_metrics)

# https://machinelearningmastery.com/save-load-keras-deep-learning-models/
network_json = network.to_json()
with open("motion3_model.json", "w") as json_file:
    json_file.write(network_json)


plt.plot(history.history['loss'])
plt.show()

print ("done")