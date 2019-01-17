from __future__ import division
from keras.models import model_from_json
# import pandas as pd
import numpy as np
# import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray




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

class RealTimeTesting(object):

    def __init__(self):

        self.ws = 50
        self.arr = np.empty((1,2))

        # load json and create model
        # https://machinelearningmastery.com/save-load-keras-deep-learning-models/
        json_file = open('model.json', 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.loaded_model = model_from_json(loaded_model_json)
        # load weights into new model
        self.loaded_model.load_weights("weights.best.hdf5")
        print("Loaded model from disk")
        # evaluate loaded model on test data
        self.loaded_model.compile(loss='mse', optimizer='RMSprop', metrics=['mse'])

        pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_sub_func)

    def pcf_sub_func(self, msg):
        # features = np.append(features, [[msg.data[0], msg.data[1]]], axis=0)
        # print (msg.data[0])
        self.arr = np.append(self.arr, [[msg.data[0], msg.data[1]]], axis=0)
        if (self.arr.shape[0] >= 50):
            ws_0 = self.arr.shape[0] - self.ws
            # print ("(" + str(0 + ws_0) + " , " + str(self.ws + ws_0) + ")")
            arr_3d = self.arr[0+ws_0 : self.ws+ws_0].reshape(1,self.ws,2)
            # print (arr_3d.shape)
            y_predict = self.loaded_model.predict(arr_3d, verbose=1, steps=None)
            print (y_predict)
            # return self.arr[0+ws_0 : self.ws+ws_0]








if __name__ == "__main__":
    rospy.init_node('real_time_testing')
    rlt = RealTimeTesting()
    # learned_model = rlt.load_model()
    rospy.spin()




# score = loaded_model.evaluate(X, Y, verbose=0)

# fig = plt.figure()
# plt.plot(np.sort(targets[:,0]), 'b', ms=1.5, label='actual')
# plt.plot(np.sort(y_predict[:,0]), 'r', ms=1.5, label='predictions')
# plt.show()
#
# print ('Done')
