#!/usr/bin/env python3
from __future__ import print_function
from keras.layers import Dense, Activation, Conv1D, MaxPooling1D, Flatten, Dropout, BatchNormalization
from keras.models import Sequential
from keras.losses import cosine_proximity, mean_squared_error
from keras import layers
from sklearn import preprocessing
from keras.activations import exponential, linear
import keras
import pandas as pd
from keras.optimizers import Adam, RMSprop
from keras import regularizers
import numpy as np
from tensorflow.losses import huber_loss
import random
from keras import backend as K
import sys
import tensorflow as tf
from sklearn.svm import SVR
import matplotlib.pyplot as plt
from scipy import signal
WAVELET_THRESHOLD=0.2
sess = tf.Session()
K.set_session(sess)
np.set_printoptions(threshold=np.nan)



def plot_history(history):
    plt.figure()
    plt.xlabel('Epoch')
    plt.ylabel('Mean Abs Error [1000$]')
    plt.plot(history.epoch, np.array(history.history['mean_absolute_error']),
           label='Train Loss')
    plt.plot(history.epoch, np.array(history.history['val_mean_absolute_error']),
           label = 'Val loss')
    plt.legend()
    plt.ylim([0, 5])
    plt.show()

def custom_loss(y_true, y_pred):
    return mean_squared_error(100*y_true, 100*y_pred)
    #a= y_true[:, 1]*100
    #b= y_pred[:, 1]*100
    #d= y_true[:, 0]
    #e= y_pred[:, 0]
    #delthet=d-e
    #cosdelt= K.cos(delthet*np.pi)

    #c =a*a + b*b - 2*a*b*cosdelt
    #res = K.mean(c)
    #return K.sqrt(res)

def custom_activation(x):
    #return K.sigmoid(x)-0.5
    return 5*K.tanh(x)
    #return exponential(x)
    #return 10*linear(x)

def model_function(data, labels, test, lab_test):

    data= data.reshape(data.shape[0], data.shape[1], 1)
    test= test.reshape(test.shape[0], test.shape[1], 1)

    model= Sequential()
    model.add(Conv1D(filters=64, kernel_size=5, input_shape=(250, 1)))
    model.add(MaxPooling1D(5))
    #model.add(Dropout(0.5))
    model.add(Conv1D(filters=32, kernel_size=5))
    model.add(MaxPooling1D(5))
    #model.add(Dropout(0.5))

    model.add(Conv1D(filters=32, kernel_size=5))
    model.add(MaxPooling1D(5))
    model.add(Flatten())
    model.add(Dense(1000, activation='relu', kernel_regularizer=regularizers.l2(0.03)))
    model.add(Dense(1000, activation= 'relu', kernel_regularizer= regularizers.l2(0.03)))
    model.add(Dense(50, activation= None, kernel_regularizer= regularizers.l2(0.01)))
    model.add(Dense(1, activation=None))

    rms= RMSprop(lr=1e-3, clipvalue= 0.5)
    model.compile(loss=custom_loss,optimizer=rms)
    history= model.fit(data, labels, batch_size=20, nb_epoch=1500,  verbose=2, validation_data=(test, lab_test))
    predictions=model.predict(test, batch_size=1)
    print(np.round(predictions, 4)-lab_test)
    plt.plot(np.round(predictions, 4)-lab_test)
    plt.show()
    ii=0
    for l in model.layers:
        print(str(l.input_shape) + ' ' + str(l.output_shape))
        #print(l.get_weights())
        #l.get_weights()[0].tofile("params/weights"+str(ii)+".txt", sep=',', format="%.7e")
        #l.get_weights()[1].tofile("params/bias"+str(ii)+".txt", sep=',', format="%.7e")
        filename=open("params/weights"+str(ii)+".txt", "w")
        filename.write(str(l.get_weights()))

        ii+=1
if __name__== '__main__':
    train=0.95
    WINDOW_SIZE=125
    NUM_ADC=2
    data= pd.read_csv('alldata.csv').values

    ind= list(np.arange(data.shape[0]))

    indexes= random.sample(range(data.shape[0]), int(data.shape[0]*train))

    missing = list(set(ind)- set(indexes))

    training_data1= data[indexes, :data.shape[1]-2]

    testing_data1 = data[missing, :data.shape[1]-2]
    training_labels1= data[indexes, -2:]
    training_labels= np.empty((0, 2))
    training_data= np.empty((0, 250))
    testing_labels= np.empty((0, 2))
    testing_data= np.empty((0, 250))
    testing_labels1= data[missing, -2:]
    for i in range(training_data1.shape[0]):
        widths= np.arange(1, 31)
        cwtmatr= signal.cwt(training_data1[i, :], signal.ricker, widths)
        if np.max(np.abs(cwtmatr))>=WAVELET_THRESHOLD:
            training_labels= np.vstack((training_labels, training_labels1[i, :]))
            training_data= np.vstack((training_data, training_data1[i, :]))

    for i in range(testing_data1.shape[0]):
        widths= np.arange(1, 31)
        cwtmatr1= signal.cwt(testing_data1[i, :], signal.ricker, widths)
        if np.max(np.abs(cwtmatr1))>=WAVELET_THRESHOLD:
            testing_labels= np.vstack((testing_labels, testing_labels1[i, :]))
            testing_data= np.vstack((testing_data, testing_data1[i, :]))
    training_labels[:, 1]/=100
    testing_labels[:, 1]/=100
    training_labels[:, 0]/=180
    testing_labels[:, 0]/=180
    model= model_function(training_data, training_labels[:, 1], testing_data, testing_labels[:, 1])
