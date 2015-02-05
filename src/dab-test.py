#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os


# load data from rtl_sdr output
data = np.fromfile("test/channel_7b_iq_8bit.dat", np.uint8)

# convert into signed data
raw_data = np.asarray(data, dtype=np.int16) - 128

# cut 200 ms of sampled data
#raw_data = raw_data[0:2048 * 1000 * 2]

# convert signed data into complex data 
complex_data = (1.0 * raw_data[::2] + 1j * raw_data[1::2]) / 128.

of = open("/tmp/fft_python.dat", "wt")

fft_len = 2048
offset = 0
while (offset + fft_len) <= complex_data.size and offset < (fft_len * 1024):
    fft_input = complex_data[offset:offset + fft_len]
    fft_result = np.fft.fft(fft_input)
    fft_result = np.hstack((fft_result[fft_len/2:], fft_result[:fft_len/2]))
    
    fft_abs = np.absolute(fft_result)
    
    for x in range(fft_result.size):
        of.write("%d %e %e\n" % (x,np.real(fft_result[x]),np.imag(fft_result[x])))
        
    of.write("\n")
    offset += fft_len
