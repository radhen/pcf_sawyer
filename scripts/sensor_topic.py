#!/usr/bin/env python
from __future__ import division, print_function

import serial
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np
from scipy.signal import detrend


NUM_ANALOG_VAL = 1 #number of values read from the serial port

def collect_data(port='/dev/ttyACM0'):
    with serial.Serial(port, 115200, timeout=0.5) as ser:
        ser.flushInput()
        # Give it some time to initialize
        data = []
        N = 1
        for i in range(N):
            data.append(ser.read(ser.inWaiting()))
            rospy.loginfo("Waiting for {} s more".format(N-i))
            rospy.sleep(1)
        print('\n'.join(filter(None, data)))
        buffer = []
<<<<<<< HEAD
        r = rospy.Rate(30)
=======
        r = rospy.Rate(100)
>>>>>>> af9a9c323932e8b2ee675f31e190ce4254671154
        while not rospy.is_shutdown():
            buffer.append(ser.read(ser.inWaiting()))
            foo = ''.join(buffer).splitlines()
            try:
                last_full_line = foo[-2]
            except IndexError:
                r.sleep()
                continue
            try:
                values = [float(i) for i in last_full_line.split()]
                if len(values) == NUM_ANALOG_VAL:
                    rospy.loginfo(values)
                    yield values
            except ValueError:
                # rospy.loginfo(last_full_line)
                print('Error')
                r.sleep()
                continue
            buffer = [foo[-1]]
            r.sleep()


def sensor_node():
    c = collect_data()
    #c = collect_data(port='/dev/ttyACM1')
    pub = rospy.Publisher('/sensor_values', Float32MultiArray, queue_size=10)
<<<<<<< HEAD
    rate = rospy.Rate(30)
=======
    rate = rospy.Rate(150)
>>>>>>> af9a9c323932e8b2ee675f31e190ce4254671154
    while not rospy.is_shutdown():

        #***** data from arduino ******#
        values = next(c)
        # print (type(values))
        # np.asarray(values)
        buffer.pop(0)
        buffer.append(values[0])
        x = detrend(buffer)
        value = [x[0]]

        #****** fake signal generation *****#
        # sample_b = np.random.uniform(low=0.0, high=0.5, size=(1, 1))
        # sample_ir = np.random.uniform(low=-0.0, high=0.6, size=(1, 1))
        # values = np.concatenate((sample_b, sample_ir), axis=1)
        # values = [values[0, 0], values[0, 1]]

        #******* publish the sensor values ****#
        msg = Float32MultiArray(
            MultiArrayLayout([MultiArrayDimension('sensor data', NUM_ANALOG_VAL, 1)], 1),
            value)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sensor_node')
    BUFFER_LEN = 80
    buffer = []
    for i in range(BUFFER_LEN):
        buffer += [0]
    s = sensor_node()
