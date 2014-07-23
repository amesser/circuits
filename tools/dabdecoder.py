#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os


# load data from rtl_sdr output
data = np.fromfile("test/channel_7b_iq_8bit.dat", np.uint8)

# convert into signed data
data = np.asarray(data, dtype=np.int16) - 128

# convert signed data into complex data 
data = (1.0 * data[::2] + 1j * data[1::2]) / 128.

# generate floating average of 64 samples to detect frame start
level_data = np.average(np.abs(data).reshape((-1,64)),axis=1)
level_time = np.arange(level_data.size,dtype=np.float) * 64. / 2.048e6

try:
    os.unlink("/tmp/plot_level.dat")
except:
    pass

np.savetxt("/tmp/plot_level.dat", np.vstack((level_time, level_data)).transpose() , fmt="%.08e %.03e")
