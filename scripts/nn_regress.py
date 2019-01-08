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

df = pd.read_excel('/home/radhen/Documents/expData/motion1/all/motion1_all_15N.xlsx',sheet_name='Sheet2',header=None)
all_15 = df.as_matrix()

F = np.sqrt(np.square(all_15[:,5]) + np.square(all_15[:,6]) + np.square(all_15[:,7]))
theta = all_15[:,6]
alpha = all_15[:,7]

# all_15 = np.column_stack((all_15[:,16], F))
target = np.column_stack((F, theta))
target = np.column_stack((target, alpha))

features = all_15[:,13:15]

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