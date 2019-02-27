#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
import os
from sklearn.model_selection import train_test_split
from keras.layers import Dense, Activation, Conv1D, MaxPooling1D, Flatten, Dropout, BatchNormalization, Dropout, Conv2D
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

def preprocessData(X):

    # ----- normalizing between 0 and 1 ----- #
    for i in range(len(X)):
        for j in range(len(data_all[0])):
            X[i][j][:,14] = (X[i][j][:,14] - min(X[i][j][:,14]))/float(max(X[i][j][:,14] - min(X[i][j][:,14]))) # baro
            X[i][j][:,15] = (X[i][j][:,15] - min(X[i][j][:,15]))/float(max(X[i][j][:,15] - min(X[i][j][:,15]))) # ir

    # ----- normalizing with 0 mean and 1 std. dev. ----- #
    # for i in range(X.shape[1]):
    #     series = Series(X[:,i])
    #     # prepare data for normalization
    #     values = series.values
    #     values = values.reshape((len(values), 1))
    #     # train the normalization
    #     scaler = StandardScaler()
    #     scaler = scaler.fit(values)
    #     print('Mean: %f, StandardDeviation: %f' % (scaler.mean_, math.sqrt(scaler.var_)))
    #     # normalize the dataset and print
    #     standardized = scaler.transform(values)
    #     # print (standardized[:,0])
    #     X[:,i] = standardized[:,0]

    return (X)


folderIN = '/home/radhen/Documents/pcf_expData/motion3/lr_bf_probeUP_dragon/'
filename = [f for f in os.listdir(folderIN) if f.endswith('.txt')]
data_roll = {} #each arr in dict a single load un-load curve for a given pitch and roll
data_pitch = {}
data_roll_cycles = {}
data_pitch_cycles = {}
for l in range(4): # num of fingers
    for i in range(1): # roll | pitch
        for j in range(3): # pitch | roll
            for k in range(5): # load-unload cycles
                data_pitch_cycles[k] = np.loadtxt(folderIN + 'bf_0' + '/lr_bf_{}_{}_{}.txt'.format(i,j,k))
                data_roll_cycles[k] = np.loadtxt(folderIN + 'lr_0' + '/lr_bf_{}_{}_{}.txt'.format(j, i, k))
            data_roll[3*l+j] = data_roll_cycles
            data_pitch[3*l+j] = data_pitch_cycles
            data_roll_cycles, data_pitch_cycles = {}, {}


for i in range(12):
    data_roll[12+i] = data_pitch[i]
data_all = {}
data_all = data_roll


# NORMALIZE BETWEEN 0 and 1
data_all_norm = preprocessData(data_all)


# FILTERING THE SIGNALS
order = 1
fs = 40.0       # sample rate, Hz
cutoff = 0.5  # desired cutoff frequency of the filter, Hz
b, a = butter_lowpass(cutoff, fs, order)
for i in range(len(data_all_norm)): # roll | pitch
    for j in range(len(data_all_norm[0])): # load-unload cycles

        baro_lowpassed = butter_lowpass_filter(data_all_norm[i][j][:,14], cutoff, fs, order)
        data_all_norm[i][j][:,14] = baro_lowpassed

        ir_lowpassed = butter_lowpass_filter(data_all_norm[i][j][:, 15], cutoff, fs, order)
        data_all_norm[i][j][:, 15] = ir_lowpassed

        data_all_norm[i][j][:, 10] = np.sqrt(np.power(data_all_norm[i][j][:, 10],2) + np.power(data_all_norm[i][j][:, 9],2) + np.power(data_all_norm[i][j][:, 8],2))
        force_lowpassed = butter_lowpass_filter(data_all_norm[i][j][:, 10], cutoff, fs, order)
        data_all_norm[i][j][:, 10] = force_lowpassed
        data_all[i][j][:, 10] /= 30.0


# for i in range(len(data_all)):
#     # plt.figure()
#     for j in range(len(data_all[0])):
#         # plt.plot(np.sort(data_all[i][j][100:,14]))
#         print (data_all[i][j].shape[0])
# # plt.show()
# print ('wait')


# dydx_ir = np.gradient(lr[:,15],1)
# dydx_ir_norm = np.abs(dydx_ir)/np.max(dydx_ir)
# peaks, _ = find_peaks(dydx_ir_norm,height=0.05)


WS = 1
count = 0
noe = 0 # number of examples
for i in range(len(data_all_norm)):
    for j in range(len(data_all_norm[0])):
        # noe += data_all[i][j].shape[0]-WS-1
        noe += data_all_norm[i][j].shape[0]
print(noe)


# PASSING DATA SAMPLE-BY-SAMPLE INTO THE NETWORK
X = np.asarray(())
Y = np.asarray(())


for i in range(len(data_all_norm)):
    for j in range(len(data_all_norm[0])):
        if (i==0 and j==0):
            X = data_all_norm[i][j][:,14:16]
            Y = data_all_norm[i][j][:,10]
        else:
            X = np.vstack((X,data_all_norm[i][j][:,14:16]))
            Y = np.hstack((Y,data_all_norm[i][j][:,10]))


# X[:,:,0] /= np.power(2,24)
# X[:,:,1] /= np.power(2,16)

train_x, test_x, train_y, test_y = train_test_split(X,Y,test_size=0.3,random_state=0)

# Start neural network
network = models.Sequential()
network.add(layers.Dense(units=2, input_dim=2, activation='relu'))
network.add(layers.Dense(units=6, activation='relu'))
network.add(layers.Dense(units=12, activation='relu'))
network.add(layers.Dense(units=4, activation='relu'))
network.add(layers.Dense(units=1))


network.compile(loss='mse', optimizer='Adam', metrics=['mse'])
print (network.summary())


# Checkpoint. Useful link: https://machinelearningmastery.com/check-point-deep-learning-models-keras/
filepath="dragon.weights.best.hdf5"
checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]


# Train neural network
history = network.fit(train_x, # Features
                      train_y, # Target vector
                      epochs=15, # Number of epochs
                      verbose=1, # No output
                      batch_size=50, # Number of observations per batch
                      validation_data=(test_x, test_y), # Data for evaluation
                      callbacks=callbacks_list)


loss_and_metrics = network.evaluate(test_x, test_y, batch_size=10)
print (loss_and_metrics)


# https://machinelearningmastery.com/save-load-keras-deep-learning-models/
network_json = network.to_json()
with open("dragon.model.json", "w") as json_file:
    json_file.write(network_json)


plt.plot(history.history['loss'])
plt.show()


print ("done")