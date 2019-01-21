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

        cm = mpl.cm.get_cmap('YlOrRd')
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(np.random.random((3,3)),
                                 cmap=cm,
                                 interpolation='none',
                                 vmin=0,
                                 vmax=1,
                                 # Doesn't seem to make a difference (?)
                                 animated=True)

        # colorbar = self.fig.colorbar(self.im, orientation='horizontal')
        # colorbar.set_ticks(np.arange(12, 17))
        # # LaTeX powers
        # colorbar.set_ticklabels([r'$2^{{{}}}$'.format(i) for i in range(12, 17)])
        # # Big numbers
        # colorbar.set_ticklabels([str(int(i)) for i in np.logspace(12, 16, 5, base=2)])

        self.ax.set_xticks([-6, -2, 2, 6])
        self.ax.set_xticklabels([-6, -2, 2, 6])
        self.ax.set_yticks([-6, -2, 2, 6])
        self.ax.set_yticklabels([-6, -2, 2, 6])
        # self.ax.set_xlim(-60, 60)
        self.ax.grid()
        self.fig.show()
        self.fig.canvas.draw()



        self.sub = rospy.Subscriber(topic, Float32MultiArray, self.callback)

    def callback(self, msg):
        # nparr = Int32MultiArray2np(msg)
        # if nparr.shape != (16,):
        #     raise ValueError("Need 3 sensor values!")
        #
        # data = nparr.reshape(2, 8)
        # self.im.set_data(np.log2(data))
        # self.ax.set_title(str(data))
        # self.fig.canvas.draw()

        self.im.set_array(np.random.random((6, 6)))
        self.fig.canvas.draw()


if __name__ == '__main__':
    vis = FingerSensorVisualizer()
    plt.show()
    rospy.spin()