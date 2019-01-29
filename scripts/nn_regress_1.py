#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz
import os


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


folderIN = '/home/radhen/Documents/expData/motion3/'
filename = [f for f in os.listdir(folderIN) if f.endswith('.txt')]
lr = {}
bf = {}
for i in range(len(filename)):
    lr[i] = np.loadtxt(folderIN + '/motion3_lr/lr_{}.txt'.format(i))
    bf[i] = np.loadtxt(folderIN + '/motion3_bf/bf_{}.txt'.format(i))


# Filter requirements.
# order = 6
# fs = 40.0       # sample rate, Hz
# cutoff = 2.0  # desired cutoff frequency of the filter, Hz
# b, a = butter_lowpass(cutoff, fs, order)
# data = lr[:,15]
# data_lowpassed = butter_lowpass_filter(data, cutoff, fs, order)

# plt.plot(lr[:,15]/np.power(2,16))
# plt.plot(lr[:,14]/np.power(2,24))

# dydx_ir = np.gradient(lr[:,15],1)
# dydx_ir_norm = np.abs(dydx_ir)/np.max(dydx_ir)
# peaks, _ = find_peaks(dydx_ir_norm,height=0.05)


print ("done")