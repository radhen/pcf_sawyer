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

# Set random seed
np.random.seed(0)

# Generate FAKE features matrix and target vector
# features, target = make_regression(n_samples = 10000,
#                                    n_features = 3,
#                                    n_informative = 3,
#                                    n_targets = 1,
#                                    noise = 0.0,
#                                    random_state = 0)

df_15 = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx',sheet_name='Sheet1',header=None)
df_20 = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx',sheet_name='Sheet2',header=None)
df_25 = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx',sheet_name='Sheet3',header=None)
df_30 = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx',sheet_name='Sheet4',header=None)
df_35 = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all.xlsx',sheet_name='Sheet5',header=None)

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

# Divide our data into training and test sets
train_features, test_features, train_target, test_target = train_test_split(features,
                                                                            target,
                                                                            test_size=0.33,
                                                                            random_state=0)

# Start neural network
network = models.Sequential()

# Add fully connected layer with a ReLU activation function
network.add(layers.Dense(units=32, activation='relu', input_shape=(train_features.shape[1],)))

# Add fully connected layer with a ReLU activation function
network.add(layers.Dense(units=32, activation='relu'))

# Add fully connected layer with no activation function
network.add(layers.Dense(units=3))

# Compile neural network
network.compile(loss='mse', # Mean squared error
                optimizer='RMSprop', # Optimization algorithm
                metrics=['mse']) # Mean squared error

# Train neural network
history = network.fit(train_features, # Features
                      train_target, # Target vector
                      epochs=10, # Number of epochs
                      verbose=0, # No output
                      batch_size=100, # Number of observations per batch
                      validation_data=(test_features, test_target)) # Data for evaluation


loss_and_metrics = network.evaluate(test_features, test_target, batch_size=128)

print ("Done")