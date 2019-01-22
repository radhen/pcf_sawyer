#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


def Int32MultiArray2np(msg):
    arr = np.array(msg.data)
    return arr

class FingerSensorVisualizer(object):
    def __init__(self, topic='/nn_predictions'):
        self.nh = rospy.init_node('Visualizer', anonymous=True)

        self.arr = np.zeros((3,3))

        cm = mpl.cm.get_cmap('YlOrRd')
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(self.arr,
                                 cmap=cm,
                                 interpolation='none',
                                 vmin=0,
                                 vmax=1,
                                 # Doesn't seem to make a difference (?)
                                 animated=True)

        colorbar = self.fig.colorbar(self.im, orientation='vertical')
        # colorbar.set_ticks(np.arange(12, 17))
        # LaTeX powers
        # colorbar.set_ticklabels([r'$2^{{{}}}$'.format(i) for i in range(12, 17)])
        # Big numbers
        # colorbar.set_ticklabels([str(int(i)) for i in np.logspace(12, 16, 5, base=2)])

        self.ax.set_xticks(range(3))
        self.ax.set_xticklabels([-40,0,40])
        self.ax.set_yticks(range(3))
        self.ax.set_yticklabels([-40,0,40])
        # self.ax.set_xlim(-60, 60)
        self.ax.grid()
        self.fig.show()
        self.fig.canvas.draw()



        self.sub = rospy.Subscriber(topic, Float32MultiArray, self.callback)

    def callback(self, msg):

        if (msg.data[1] <= 0.33):
            i = 0
        if (0.33 < msg.data[1] <= 0.66):
            i = 1
        if (0.66 < msg.data[1] <= 0.1):
            i = 2
        if (msg.data[2] <= 0.33):
            j = 0
        if (0.33 < msg.data[2] <= 0.66):
            j = 1
        if (0.66 < msg.data[2] <= 0.1):
            j = 2

        self.arr[i,j] = msg.data[0]
        self.im.set_data(self.arr)

        self.im.set_data(np.array([[0, 0, 0],
                                   [0, msg.data[0], 0],
                                   [0, 0, 0]]))

        self.fig.canvas.draw()



if __name__ == '__main__':
    vis = FingerSensorVisualizer()
    plt.show()
    rospy.spin()