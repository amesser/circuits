#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os


# load data from rtl_sdr output
data = np.fromfile("test/channel_7b_iq_8bit.dat", np.uint8)

# convert into signed data
raw_data = np.asarray(data, dtype=np.int16) - 128

# cut 200 ms of sampled data
raw_data = raw_data[0:2048 * 1000 * 2]

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

offset_start = None

fft_len      = 2048
guard_length = 504
carriers     = 1536

frame_length = 0.096

sample_rate = 2048000


def mapping_generator(fft_len, carriers):
    lower_bound = (fft_len - carriers) / 2
    upper_bound = fft_len - lower_bound
    
    v1 = fft_len - carriers - 1
    x = 0
    for i in xrange(fft_len):
        if x != fft_len / 2 and x >= lower_bound and x <= upper_bound: 
            yield x
            
        x = ((13 * x) + v1) % fft_len
        
mapping = np.fromiter(mapping_generator(fft_len, carriers),dtype=np.int, count=carriers) 
        
#                   int16_t lwb, int16_t upb, int16_t *v) {
#int16_t    *tmp    = (int16_t *)alloca (T_u * sizeof (int16_t));
#int16_t    index    = 0;
#int16_t    i;
#
#    tmp [0]    = 0;
#    for (i = 1; i < T_u; i ++)
#       tmp [i] = (13 * tmp [i - 1] + V1) % T_u;
#    for (i = 0; i < T_u; i ++) {
#       if (tmp [i] == T_u / 2)
#          continue;
#       if ((tmp [i] < lwb) ||
#           (tmp [i] > upb)) 
#          continue;
#//    we now have a table with values from lwb .. upb
#//
#       v [index ++] = tmp [i] - T_u / 2;
#//    we now have a table with values from lwb - T_u / 2 .. lwb + T_u / 2
#    }
#
#    return v;




fft_data   = None
fft_center = None

correction_steps  = fft_len * 1024
correction_table  = np.sin( 2.0 * np.pi * np.arange(correction_steps) / correction_steps)
correction_factor = 0


correction_phase = 0

state   = 0
offset  = 0
symbol_cnt = 0

while (offset + fft_len + guard_length) <= abs_data.size:
    if state == 0 and moving_average[offset] < (0.5 * pt1_average[offset]) and offset > moving_len:
        state = 1
    elif state == 1 and moving_average[offset] > pt1_average[offset]:
        # Frame Start Detected
        print ("NULL")
        
        state = 0
        offset_start = offset
        frame_start[offset_start] = 0.03
        
        
        if symbol_cnt:          
            symbol_cnt = 0
                                         
    if offset_start is not None:
        x = offset - offset_start
        modulo = fft_len + guard_length
        
        if (x % modulo == 0) and symbol_cnt < 76:
            symbol_cnt += 1
             
            sinus_index = (np.arange(0,modulo) * correction_factor) + correction_phase
            correction_phase = sinus_index[-1] + correction_factor
            
            sinus_index   = sinus_index % correction_steps
            cosinus_index =  (sinus_index + correction_steps/4) % correction_steps            
            
            correction = correction_table[cosinus_index] + 1.0j * correction_table[sinus_index] 
                        
            samples    = complex_data[offset:offset + modulo] * correction
            
            fft_input = samples[0:fft_len]
            
            fft_result = np.fft.fft(fft_input)
            
            fft_result = np.hstack((fft_result[fft_len/2:], fft_result[:fft_len/2]))
            
            fft_abs = np.abs(fft_result)
            
            
            if (x / 2048) < 4:
                 
                if fft_data is None:
                    fft_data = fft_result
                else:
                    fft_data = np.vstack((fft_data, fft_result))
                    
            carrier_detect = np.nonzero((np.average(fft_abs)) < fft_abs)[0]            
            center = (carrier_detect[0] + carrier_detect[-1]) / 2.
                
            if symbol_cnt == 1:
                delta_f = (float(sample_rate) / float(fft_len)) * (center - fft_len/2)
                
                if abs(delta_f) > 1000:
                    correction_factor = (correction_factor - (correction_steps * delta_f / sample_rate))
                    
            if abs(delta_f) < 1000 and symbol_cnt > 1:
                delta_f_fine = sample_rate / fft_len * np.average(np.angle(samples[fft_len:] * np.conjugate(samples[:guard_length]))) / 2. / np.pi
                
                correction_factor = 0.9 * correction_factor + 0.1 * (correction_factor - (correction_steps * delta_f_fine / sample_rate))
            else:
                delta_f_fine = None
                
            correction_factor = int(correction_factor)
            
            if delta_f < 1000:
                if symbol_cnt == 1:
                    # phase reference symbol
                    phase_reference = fft_result
                else:
                    ofdm_data = fft_result[mapping] * np.conjugate(fft_result[mapping])
                    phsae_reference = fft_result
                    
                    ofdm_data = ofdm_data * 255 / np.abs(ofdm_data)
                    
                    bits = np.hstack((np.real(ofdm_data), np.imag(ofdm_data)))
                    
                if symbol_cnt == 2:
                    ofdm_fic_data = bits
                elif symbol_cnt > 2 and symbol_cnt <= 4:
                    ofdm_fic_data = np.hstack((ofdm_fic_data, bits))
                    
                if symbol_cnt >= 2:
                    while ofdm_fic_data.size >= 2304:
                        viterby_input_data  = ofdm_fic_data[:2304]
                        ofdm_fic_data       = ofdm_fic_data[2304:]
                        
                          
            

                            
            print (center, delta_f, delta_f_fine, correction_factor, symbol_cnt)
                
            # calculate frequency deviation
                #correction_factor += (fft_len / 2 - center) * correction_steps / 2 / np.pi
               
                #print correction_factor
        else:
            correction_phase = correction_phase + correction_factor


    
    offset = offset + 1     
                
                           
            
            

try:
    os.unlink("/tmp/plot_null.dat")
except:
    pass

np.savetxt("/tmp/plot_null.dat", np.vstack((null_time, pt1_average, moving_average, frame_start)).transpose() , fmt="%.08e %.03e %.03e %.03e")

try:
    os.unlink("/tmp/plot_fft.dat")
except:
    pass

np.savetxt("/tmp/plot_fft.dat", np.abs(fft_data).transpose() , fmt="%.03e")

