from __future__ import division
from keras.models import model_from_json
# import pandas as pd
import numpy as np
# import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray


rospy.init_node('node_name')

# filename = '/home/radhen/Documents/expData/motion1/all/20deg/motion1_all.xlsx'
# df_15 = pd.read_excel(filename,sheet_name='Sheet1',header=None)
#
# all_15 = df_15.as_matrix()
# all = all_15[950:,:]
#
# # Initial definations of targets and features for the problem
# F = np.sqrt(np.square(all[:,8]) + np.square(all[:,9]) + np.square(all[:,10]))  #resultant force F = sqrt(Fx^2+Fy^2+Fz^2)
# theta = all[:,6]
# alpha = all[:,7]
# targets = np.column_stack((F, theta))
# targets = np.column_stack((targets, alpha)) # [F, theta, alpha]
# features = all[:,13:15] # [baro, ir]
#
# # Re-shaping after train test split to create a buffer of window size WS (below)
# WS = 50
# test_x = np.zeros((features.shape[0], WS, 2))
# for i in range(features.shape[0]-WS-1):
#     test_x[i, :, :] = features[i:WS+i,:] #this is where the window is actually moving

# load json and create model
json_file = open('model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)

# load weights into new model
loaded_model.load_weights("weights.best.hdf5")
print("Loaded model from disk")

# evaluate loaded model on test data
loaded_model.compile(loss='mse', # Mean squared error
                optimizer='RMSprop', # Optimization algorithm
                metrics=['mse']) # Mean squared error

# y_predict = loaded_model.predict(test_x, batch_size=10, verbose=0, steps=None)
features = np.empty((1,2))


def pcf_sub_func(msg):
    features = np.append(features, [[msg.data[0], msg.data[1]]], axis=0)
    print (features.shape)



while (1):
    pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, pcf_sub_func)



# score = loaded_model.evaluate(X, Y, verbose=0)

# fig = plt.figure()
# plt.plot(np.sort(targets[:,0]), 'b', ms=1.5, label='actual')
# plt.plot(np.sort(y_predict[:,0]), 'r', ms=1.5, label='predictions')
# plt.show()
#
# print ('Done')
