#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os


# load data from rtl_sdr output
data = np.fromfile("test/channel_7b_iq_8bit.dat", np.uint8)

# convert into signed data
raw_data = np.asarray(data, dtype=np.int16) - 128

raw_data = raw_data[0:2048 * 100 * 2]

# convert signed data into complex data 
complex_data = (1.0 * raw_data[::2] + 1j * raw_data[1::2]) / 128.

# generate floating average of 64 samples to detect frame start
level_data = np.average(np.abs(complex_data).reshape((-1,64)),axis=1)
level_time = np.arange(level_data.size,dtype=np.float) * 64. / 2.048e6

try:
    os.unlink("/tmp/plot_level.dat")
except:
    pass

np.savetxt("/tmp/plot_level.dat", np.vstack((level_time, level_data)).transpose() , fmt="%.08e %.03e")

# detect starting points
pt1_length = 10000
moving_len = 500
 
abs_data        = np.abs(complex_data)
pt1_average     = np.zeros_like(abs_data)
moving_average = np.zeros_like(abs_data)

for offset in xrange(1,abs_data.size):
    pt1_average[offset] = (pt1_average[offset-1] * (pt1_length-1) + abs_data[offset]) / pt1_length

for offset in xrange(moving_len,abs_data.size):
    moving_average[offset - 1] = np.average(abs_data[offset-moving_len:offset])

null_time = np.arange(abs_data.size,dtype=np.float) / 2.048e6

frame_start = np.zeros_like(abs_data)

state = 0
for offset in xrange(0,abs_data.size):
    if state == 0 and moving_average[offset] < (0.5 * pt1_average[offset]):
        state = 1
    elif state == 1 and moving_average[offset] > pt1_average[offset]:
        state = 0
        frame_start[offset] = 0.03
        
    

            

try:
    os.unlink("/tmp/plot_null.dat")
except:
    pass

np.savetxt("/tmp/plot_null.dat", np.vstack((null_time, pt1_average, moving_average, frame_start)).transpose() , fmt="%.08e %.03e %.03e %.03e")
