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
        self.pcf_data = np.zeros((1,4))
        self.endeff_data = np.zeros((1,15))

    def start_recording(self):
        self.pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_callback)
        self.ft_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.endeff_callback)

    def pcf_callback(self, data):
        ################## TIME ######################
        now = rospy.get_rostime()
        sec = now.secs
        nsec = now.nsecs
        ################## PCF ######################
        baro = data.data[0]
        ir = data.data[1]
        self.pcf_data = np.append(self.pcf_data, np.array([[sec, nsec, baro, ir]]), axis=0)

    def endeff_callback(self, data):
        ################## TIME ######################
        sec = data.header.stamp.secs
        nsec = data.header.stamp.nsecs
        ################## POSE ######################
        px = data.pose.position.x
        py = data.pose.position.y
        pz = data.pose.position.z
        ox = data.pose.orientation.x
        oy = data.pose.orientation.y
        oz = data.pose.orientation.z
        ow = data.pose.orientation.w
        ################## ForceTorque ######################
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        tx = data.wrench.torque.x
        ty = data.wrench.torque.y
        tz = data.wrench.torque.z

        self.endeff_data = np.append(self.endeff_data, np.array([[sec, nsec, px, py, pz, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz]]), axis=0)

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

        # self.pcf_baro = np.array(self.pcf_baro)
        np.savetxt(path+'/pcf_data.txt', self.pcf_data)

        # self.fx = np.array(self.fx)
        np.savetxt(path+'/endeff_data.txt', self.endeff_data)



if __name__ == "__main__":
    rospy.init_node('get_data')
    gd = GetData()
    gd.start_recording()
    rospy.sleep(3)
    gd.stop_recording()
    gd.convertandsave()
