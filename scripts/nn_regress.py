#!/usr/bin/env python
import numpy as np
from keras.preprocessing.text import Tokenizer
from keras import models
from keras import layers
from keras.callbacks import ModelCheckpoint
from keras.models import model_from_json
from sklearn.datasets import make_regression
from sklearn.model_selection import train_test_split
from sklearn import preprocessing
import tensorflow as tf
import pandas as pd
import matplotlib.pyplot as plt
from keras.layers import Dense, Activation, Conv1D, MaxPooling1D, Flatten, Dropout, BatchNormalization

from pandas import Series
from sklearn.preprocessing import StandardScaler
import math

# Set random seed
np.random.seed(0)


# TODO Clean all excel file reading stuff
filename = '/home/radhen/Documents/expData/motion1/all/30deg/all_30d.xlsx'
df_30_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
df_30_20 = pd.read_excel(filename,sheet_name='Sheet2',header=None)
df_30_25 = pd.read_excel(filename,sheet_name='Sheet3',header=None)
df_30_30 = pd.read_excel(filename,sheet_name='Sheet4',header=None)

all_30_15 = df_30_15.as_matrix()
# all_15 = all_15[950:,:]
all_30_20 = df_30_20.as_matrix()
# all_20 = all_20[750:,:]
all_30_25 = df_30_25.as_matrix()
# all_25 = all_25[150:,:]
all_30_30 = df_30_30.as_matrix()
# all_30 = all_30[350:,:]

filename = '/home/radhen/Documents/expData/motion1/all/20deg/all_20d.xlsx'
df_20_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
df_20_20 = pd.read_excel(filename,sheet_name='Sheet2',header=None)
df_20_25 = pd.read_excel(filename,sheet_name='Sheet3',header=None)
df_20_30 = pd.read_excel(filename,sheet_name='Sheet4',header=None)

all_20_15 = df_20_15.as_matrix()
# all_15 = all_15[950:,:]
all_20_20 = df_20_20.as_matrix()
# all_20 = all_20[750:,:]
all_20_25 = df_20_25.as_matrix()
# all_25 = all_25[150:,:]
all_20_30 = df_20_30.as_matrix()
# all_30 = all_30[350:,:]


all_30 = all_30_15
all_30 = np.append(all_30,all_30_20,axis=0)
all_30 = np.append(all_30,all_30_25,axis=0)
all_30 = np.append(all_30,all_30_30,axis=0)

all_20 = all_20_15
all_20 = np.append(all_20,all_30_20,axis=0)
all_20 = np.append(all_20,all_30_25,axis=0)
all_20 = np.append(all_20,all_30_30,axis=0)

filename = '/home/radhen/Documents/expData/motion1/left/45deg/left_45d.xlsx'
df_45_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
df_45_20 = pd.read_excel(filename,sheet_name='Sheet2',header=None)
df_45_25 = pd.read_excel(filename,sheet_name='Sheet3',header=None)
df_45_30 = pd.read_excel(filename,sheet_name='Sheet4',header=None)

all_45_15 = df_45_15.as_matrix()
# all_15 = all_15[950:,:]
all_45_20 = df_45_20.as_matrix()
# all_20 = all_20[750:,:]
all_45_25 = df_45_25.as_matrix()
# all_25 = all_25[150:,:]
all_45_30 = df_45_30.as_matrix()
# all_30 = all_30[350:,:]

all_45 = all_45_15
all_45 = np.append(all_45,all_45_20,axis=0)
all_45 = np.append(all_45,all_45_25,axis=0)
all_45 = np.append(all_45,all_45_30,axis=0)

filename = '/home/radhen/Documents/expData/motion1/right/45deg/right_45d.xlsx'
df_45r_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
df_45r_20 = pd.read_excel(filename,sheet_name='Sheet2',header=None)
df_45r_25 = pd.read_excel(filename,sheet_name='Sheet3',header=None)
df_45r_30 = pd.read_excel(filename,sheet_name='Sheet4',header=None)

all_45r_15 = df_45r_15.as_matrix()
# all_15 = all_15[950:,:]
all_45r_20 = df_45r_20.as_matrix()
# all_20 = all_20[750:,:]
all_45r_25 = df_45r_25.as_matrix()
# all_25 = all_25[150:,:]
all_45r_30 = df_45r_30.as_matrix()
# all_30 = all_30[350:,:]

all_45r = all_45r_15
all_45r = np.append(all_45r,all_45r_20,axis=0)
all_45r = np.append(all_45r,all_45r_25,axis=0)
all_45r = np.append(all_45r,all_45r_30,axis=0)

all = all_20
all = np.append(all,all_30,axis=0)
all = np.append(all,all_45,axis=0)
all = np.append(all,all_45r,axis=0)


print ('Number of DATA points = '+str(all.shape[0]))

# Initial definations of targets and features for the problem
F = np.sqrt(np.square(all[:,8]) + np.square(all[:,9]) + np.square(all[:,10]))  #resultant force F = sqrt(Fx^2+Fy^2+Fz^2)
theta = all[:,5]
alpha = all[:,6]
targets = np.column_stack((F, theta))
targets = np.column_stack((targets, alpha)) # [F, theta, alpha]
features = all[:,14:16] # [baro, ir]


# Plotting stuff
# plt.plot(target[:,0], 'b', ms=1.5, label='Measurement')
# plt.show()


def preprocessData(X):
    '''normalizing or standardize
    '''

    # ----- normalizing between 0 and 1 ----- #
    for i in range(X.shape[1]):
        X[:,i] = (X[:,i] - min(X[:,i])) / float(max(X[:,i]) - min(X[:,i]))

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

features = preprocessData(features)
targets = preprocessData(targets)

# Divide our data into training and test sets.
# Splitting this way selects points at random from the dataset. Does it matter? Does sorting the data matter
train_features, test_features, train_targets, test_targets = train_test_split(features,
                                                                            targets,
                                                                            test_size=0.2,
                                                                            random_state=0)

# Re-shaping after train test split to create a buffer of window size WS (below)
WS = 50
train_x = np.zeros((train_features.shape[0], WS, 2))
for i in range(train_features.shape[0]-WS-1):
    train_x[i, :, :] = train_features[i:WS+i,:] #this is where the window is actually moving

test_x = np.zeros((test_features.shape[0], WS, 2))
for i in range(test_features.shape[0]-WS-1):
    test_x[i, :, :] = test_features[i:WS + i, :]


# Start neural network
network = models.Sequential()
network.add(Conv1D(filters=8, kernel_size=4, input_shape=(WS, 2)))
# network.add(MaxPooling1D(5))
network.add(Conv1D(filters=8, kernel_size=4))
network.add(Flatten())
network.add(layers.Dense(units=64, activation='relu'))
network.add(layers.Dense(units=42, activation='relu'))
network.add(layers.Dense(units=28, activation='relu'))
network.add(layers.Dense(units=18, activation='relu'))
network.add(layers.Dense(units=12, activation='relu'))
network.add(layers.Dense(units=8, activation='relu'))
network.add(layers.Dense(units=5, activation='relu'))
network.add(layers.Dense(units=3))

# network.load_weights("weights.best.hdf5")

network.compile(loss='mse', optimizer='RMSprop', metrics=['mse'])

print (network.summary())

# Checkpoint. Useful link: https://machinelearningmastery.com/check-point-deep-learning-models-keras/
filepath="weights.best.hdf5"
checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

# Train neural network
history = network.fit(train_x, # Features
                      train_targets, # Target vector
                      epochs=100, # Number of epochs
                      verbose=1, # No output
                      batch_size=25, # Number of observations per batch
                      validation_data=(test_x, test_targets), # Data for evaluation
                      callbacks=callbacks_list)

loss_and_metrics = network.evaluate(test_x, test_targets, batch_size=10)
print (loss_and_metrics)

# https://machinelearningmastery.com/save-load-keras-deep-learning-models/
network_json = network.to_json()
with open("model.json", "w") as json_file:
    json_file.write(network_json)


#y_predict = network.predict(test_x, batch_size=10, verbose=0, steps=None)

# fig = plt.figure()
# plt.plot(np.sort(test_targets[:,0]), 'b', ms=1.5, label='actual')
# plt.plot(np.sort(y_predict[:,0]), 'r', ms=1.5, label='predictions')
# plt.show()

print ('Done')

