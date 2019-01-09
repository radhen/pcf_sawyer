# Load libraries
import numpy as np
from keras.preprocessing.text import Tokenizer
from keras import models
from keras import layers
from sklearn.datasets import make_regression
from sklearn.model_selection import train_test_split
from sklearn import preprocessing

import pandas as pd
import matplotlib.pyplot as plt
from keras.layers import Dense, Activation, Conv1D, MaxPooling1D, Flatten, Dropout, BatchNormalization

# Set random seed
np.random.seed(0)

# Generate FAKE features matrix and target vector
# features, target = make_regression(n_samples = 10000,
#                                    n_features = 3,
#                                    n_informative = 3,
#                                    n_targets = 1,
#                                    noise = 0.0,
#                                    random_state = 0)

# TODO Clean all excel file reading part
filename = '/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx'
df_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
df_20 = pd.read_excel(filename,sheet_name='Sheet2',header=None)
df_25 = pd.read_excel(filename,sheet_name='Sheet3',header=None)
df_30 = pd.read_excel(filename,sheet_name='Sheet4',header=None)
df_35 = pd.read_excel(filename,sheet_name='Sheet5',header=None)

all_15 = df_15.as_matrix()
all_15 = all_15[950:,:]
all_20 = df_20.as_matrix()
all_20 = all_20[750:,:]
all_25 = df_25.as_matrix()
all_25 = all_25[150:,:]
all_30 = df_30.as_matrix()
all_30 = all_30[350:,:]
all_35 = df_35.as_matrix()
all_35 = all_35[500:,:]

all = all_15
all = np.append(all,all_20,axis=0)
all = np.append(all,all_25,axis=0)
all = np.append(all,all_30,axis=0)
all = np.append(all,all_35,axis=0)



F = np.sqrt(np.square(all[:,8]) + np.square(all[:,9]) + np.square(all[:,10])) # F = sqrt(Fx^2+Fy^2+Fz^2)
theta = all[:,6]
alpha = all[:,7]

target = np.column_stack((F, theta))
target = np.column_stack((target, alpha)) # [F, theta, alpha]
features = all[:,13:15] # [baro, ir]

# plt.plot(target[:,0], 'b', ms=1.5, label='Measurement')
# plt.show()

# ADD NORMALIZING CODE

# Divide our data into training and test sets. TODO:
train_features, test_features, train_target, test_target = train_test_split(features,
                                                                            target,
                                                                            test_size=0.33,
                                                                            random_state=0)

# re-shaping after train test split
WS = 50
# no_of_examples = train_features.shape[0]/WINDOW_SIZE
train_x = np.zeros((train_features.shape[0], WS, 2))
# train_y = np.zeros((no_of_examples, WINDOW_SIZE, 3))

for i in range(train_features.shape[0]-WS-1):
    # print (i)
    train_x[i, :, :] = train_features[i:WS+i,:] #this is where the window is actually moving
    # train_y[i, :, :] = train_target[WINDOW_SIZE*i:WINDOW_SIZE*(i + 1),:]

# no_of_examples = test_features.shape[0]/WINDOW_SIZE
# test_x = np.zeros((no_of_examples, WINDOW_SIZE, 2))
# test_y = np.zeros((no_of_examples, WINDOW_SIZE, 3))
# for i in range(no_of_examples):
#     test_x[i, :, :] = test_features[WINDOW_SIZE*i:WINDOW_SIZE*(i+1),:]
#     test_y[i, :, :] = test_target[WINDOW_SIZE*i:WINDOW_SIZE*(i+1), :]

test_x = np.zeros((test_features.shape[0], WS, 2))
for i in range(test_features.shape[0]-WS-1):
    test_x[i, :, :] = test_features[i:WS + i, :]


# Start neural network
network = models.Sequential()

network.add(Conv1D(filters=2, kernel_size=10, input_shape=(50, 2)))

# network.add(MaxPooling1D(5))

network.add(Conv1D(filters=2, kernel_size=10))

network.add(Flatten())

network.add(layers.Dense(units=32, activation='relu'))

network.add(layers.Dense(units=32, activation='relu'))

network.add(layers.Dense(units=3))

network.compile(loss='mse', # Mean squared error
                optimizer='RMSprop', # Optimization algorithm
                metrics=['mse']) # Mean squared error

# Train neural network
history = network.fit(train_x, # Features
                      train_target, # Target vector
                      epochs=100, # Number of epochs
                      verbose=0, # No output
                      batch_size=20, # Number of observations per batch
                      validation_data=(test_x, test_target)) # Data for evaluation


loss_and_metrics = network.evaluate(test_x, test_target, batch_size=10)

print (loss_and_metrics)