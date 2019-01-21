from __future__ import division
from keras.models import model_from_json
# import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import time
import tensorflow as tf


class RealTimeTesting(object):

    def __init__(self):

        self.ws = 50
        self.arr = np.empty((1,2))
        self.arr_3d = np.empty((1,50,2))

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.im = self.ax.imshow(np.random.random((3, 3)))
        # plt.show(block=False)
        self.fig.show()
        self.fig.canvas.draw()


    def pcf_sub_func(self, msg, loaded_model):
        # print (msg.data[0])
        self.arr = np.append(self.arr, [[msg.data[0], msg.data[1]]], axis=0)
        if (self.arr.shape[0] >= 50):
            ws_0 = self.arr.shape[0] - self.ws
            # print ("(" + str(0 + ws_0) + " , " + str(self.ws + ws_0) + ")")
            self.arr_3d = self.arr[0+ws_0 : self.ws+ws_0].reshape(1,self.ws,2)
            with graph.as_default():
                y_predict = loaded_model.predict(self.arr_3d)
            # print (y_predict)
            self.im.set_array(np.random.random((3, 3)))
            self.fig.canvas.draw()
            plt.show()

    def listener(self, loaded_model):
        pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_sub_func, loaded_model)
        rospy.spin()


if __name__ == "__main__":

    rospy.init_node('real_time_testing')

    rlt = RealTimeTesting()

    # load json and create model
    # https://machinelearningmastery.com/save-load-keras-deep-learning-models/
    json_file = open('model.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)
    # load weights into new model
    loaded_model.load_weights("weights.best.hdf5")
    print("Loaded model from disk")
    # evaluate loaded model on test data
    loaded_model.compile(loss='mse', optimizer='RMSprop', metrics=['mse'])

    # keras issue: solution https://github.com/matsui528/sis/issues/1
    global graph
    graph = tf.get_default_graph()

    rlt.listener(loaded_model)

    # rate = rospy.Rate(35)
    # while not rospy.is_shutdown():
    #     rospy.Subscriber("/sensor_values", Float32MultiArray, rlt.pcf_sub_func)
    #     y_predict = loaded_model.predict(rlt.arr_3d)
    #     print (y_predict)
    #     # im.set_array(np.random.random((3, 3)))
    #     # fig.canvas.draw()
    #     rate.sleep()


    # arr = np.random.rand(50,2)
    # arr_3d = arr.reshape(1,50,2)
    # y = rlt.loaded_model.predict(arr_3d)
    # print (y)

    # rospy.spin()