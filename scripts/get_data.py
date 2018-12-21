from __future__ import division

import os

import rospy
from std_msgs.msg import Float32MultiArray
from intera_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState

import numpy as np
# from scipy import signal
import matplotlib.pyplot as plt

from message_filters import ApproximateTimeSynchronizer, Subscriber


class GetData(object):
    def __init__(self):
        self.pcf_data = np.zeros((1,4))
        self.endeff_data = np.zeros((1,15))
        self.data = np.zeros((1,17))


    def start_recording(self):
        # self.pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_callback)
        # self.ft_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.endeff_callback)

        # MESSAGE FILTERS: http://docs.ros.org/api/message_filters/html/python/#message_filters.TimeSynchronizer
        self.pcf_sub = Subscriber("/sensor_values", Float32MultiArray)
        self.ft_sub = Subscriber("/robot/limb/right/endpoint_state", EndpointState)

        ats = ApproximateTimeSynchronizer([self.pcf_sub, self.ft_sub], queue_size=5, slop=0.01, allow_headerless=True)
        ats.registerCallback(self.got_data)


    def got_data(self, float32MultiArray, endpointState):
        print float32MultiArray.data[0]
        # print endpointState.pose.position.x

        ################## TIME ######################
        sec = endpointState.header.stamp.secs
        nsec = endpointState.header.stamp.nsecs
        ################## POSE ######################
        px = endpointState.pose.position.x
        py = endpointState.pose.position.y
        pz = endpointState.pose.position.z
        ox = endpointState.pose.orientation.x
        oy = endpointState.pose.orientation.y
        oz = endpointState.pose.orientation.z
        ow = endpointState.pose.orientation.w
        ################## ForceTorque ###############
        fx = endpointState.wrench.force.x
        fy = endpointState.wrench.force.y
        fz = endpointState.wrench.force.z
        tx = endpointState.wrench.torque.x
        ty = endpointState.wrench.torque.y
        tz = endpointState.wrench.torque.z
        ############### PCF data #####################
        baro = float32MultiArray.data[0]
        ir = float32MultiArray.data[1]

        self.data = np.append(self.data, np.array([[sec, nsec, px, py, pz, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz, baro, ir]]), axis=0)


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


    def convertandsave(self,i):
        # convert to numpy array and save
        path = '/home/radhen/Documents/expData'

        # np.savetxt(path+'/pcf_data_{}.txt'.format(i), self.pcf_data)
        # np.savetxt(path+'/endeff_data_{}.txt'.format(i), self.endeff_data)

        np.savetxt(path + '/data_{}.txt'.format(i), self.data)






if __name__ == "__main__":
    rospy.init_node('get_data')
    gd = GetData()
    # while 1:
    gd.start_recording()
    rospy.sleep(3)
    gd.stop_recording()
    gd.convertandsave('test')
    # rospy.spin()
