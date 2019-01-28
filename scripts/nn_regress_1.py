#!/usr/bin/env python
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz, find_peaks



def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


filename = '/home/radhen/Documents/expData/motion3/back_front.xlsx'
df_20_lr = pd.read_excel(filename,sheet_name='Sheet1',header=None)

lr = df_20_lr.as_matrix()

# Filter requirements.
order = 6
fs = 40.0       # sample rate, Hz
cutoff = 2.0  # desired cutoff frequency of the filter, Hz
b, a = butter_lowpass(cutoff, fs, order)

data = lr[:,15]
data_lowpassed = butter_lowpass_filter(data, cutoff, fs, order)

# fs=40.0
# cutoff = 4.0  # desired cutoff frequency of the filter, Hz
# b, a = butter_highpass(cutoff, fs, order)
# data_highpassed = butter_highpass_filter(data_lowpassed, cutoff, fs, order)

plt.plot(lr[:,15]/np.power(2,16))
plt.plot(lr[:,14]/np.power(2,24))

dydx_ir = np.gradient(lr[:,15],1)
dydx_ir_norm = np.abs(dydx_ir)/np.max(dydx_ir)
peaks, _ = find_peaks(dydx_ir_norm,height=0.05)

plt.plot(dydx_ir_norm)
plt.plot(peaks, dydx_ir_norm[peaks], "x")
plt.show()


print ("done")