from __future__ import division

import os

import rospy
from std_msgs.msg import Float32MultiArray
from intera_core_msgs.msg import EndpointState

import numpy as np
# from scipy import signal
import matplotlib.pyplot as plt


class GetData(object):
    def __init__(self):
        self.pcf_baro = []
        self.fx = []
        self.time1 = []
        self.time2 = []

    def start_recording(self):
        self.pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_callback)
        self.ft_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.ft_callback)

    def pcf_callback(self, data):
        self.pcf_baro.append(data.data[0])

    def ft_callback(self, data):
        self.time2.append(data.header.stamp.secs)
        self.fx.append(data.wrench.force.x)

    def stop_recording(self):
        self.pcf_sub.unregister()
        self.ft_sub.unregister()

    def load_data(self):
        # load recorded data
        self.acc_x = np.loadtxt('acc_x.txt')
        # print type(self.acc_x)
        self.acc_y = np.loadtxt('acc_y.txt')
        self.acc_z = np.loadtxt('acc_z.txt')
        self.values = np.loadtxt('values.txt')
        self.gripperAperture = np.loadtxt('gripperAperture.txt')
        self.centeringerr = np.loadtxt('centeringerr.txt')

    def convertandsave(self):
        # convert to numpy array and save
        path = '/home/radhen/Documents/expData'

        self.pcf_baro = np.array(self.pcf_baro)
        np.savetxt(path+'/pcf_baro.txt', self.pcf_baro)

        self.fx = np.array(self.fx)
        np.savetxt(path+'/fy.txt', self.fx)



if __name__ == "__main__":
    rospy.init_node('get_data')
    gd = GetData()
    gd.start_recording()
    rospy.sleep(3)
    gd.stop_recording()
    gd.convertandsave()
