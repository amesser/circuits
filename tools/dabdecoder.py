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

# generate floating average of 64 samples to detect frame start
#level_data = np.average(np.abs(complex_data).reshape((-1,64)),axis=1)
#level_time = np.arange(level_data.size,dtype=np.float) * 64. / 2.048e6

#try:
#    os.unlink("/tmp/plot_level.dat")
#except:
#    pass

#np.savetxt("/tmp/plot_level.dat", np.vstack((level_time, level_data)).transpose() , fmt="%.08e %.03e")

# detect starting points
pt1_length = 10000
moving_len = 500
 
abs_data        = np.absolute(complex_data)


#pt1_average     = np.zeros_like(abs_data)
#for offset in xrange(1,abs_data.size):
#    pt1_average[offset] = (pt1_average[offset-1] * (pt1_length-1) + abs_data[offset]) / pt1_length

#moving_average = np.zeros_like(abs_data)

#for offset in xrange(moving_len,abs_data.size):
#    moving_average[offset - 1] = np.average(abs_data[offset-moving_len:offset])

a = abs_data
b = np.zeros_like(a)
b[pt1_length:] = -a[:-pt1_length]

pt1_average = np.add.accumulate(a + b) / pt1_length


a = abs_data
b = np.zeros_like(a)
b[moving_len:] = -a[:-moving_len]

moving_average = np.add.accumulate(a + b) / moving_len

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
        

puncturing_vectors = {
  1      : [0, 1,    4,       8,        12,         16,         20,         24,         28],
  8      : [0, 1,    4, 5,    8, 9,     12, 13,     16, 17,     20, 21,     24, 25,     28, 29],
  15     : [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18, 20, 21, 22, 24, 25, 26, 28, 29],
  16     : [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18, 20, 21, 22, 24, 25, 26, 28, 29, 30],
  "tail" : [0, 1,    4, 5,    8, 9,     12, 13,     16, 17,     20, 21]
}

depuncture_fic = \
    np.hstack((np.tile(puncturing_vectors[16], 21 * 4),
               np.tile(puncturing_vectors[15], 3  * 4),
               puncturing_vectors["tail"])) +\
    np.hstack((np.repeat(np.arange(0,21 * 4)  * 32, len(puncturing_vectors[16])),
               np.repeat(np.arange(21 * 4,24 * 4) * 32, len(puncturing_vectors[15])),
               np.repeat(np.arange(24 * 4,24 * 4 + 1) * 32, len(puncturing_vectors["tail"]))))
    
from puncturing import puncturing
fic_puncturer = puncturing()
fic_puncturer.mask = np.zeros(3072 + 24, dtype = np.bool8)
fic_puncturer.mask[depuncture_fic] = 1
fic_puncturer.mask = fic_puncturer.mask.reshape((-1,4))


def scrambler(length):
    value = 0x1FF;
    
    for x in range(length):
        result = 0 if (0x110 & value) in (0, 0x110) else 1
        value = value << 1 | result
        yield result
         
descramble = np.fromiter(scrambler(1024),dtype=np.int, count=1024)

fft_data   = None
fft_center = None

correction_steps  = fft_len * 1024
correction_table  = np.sin( 2.0 * np.pi * np.arange(correction_steps) / correction_steps)
correction_factor = 0


correction_phase = 0

state   = 0
offset  = 0
symbol_cnt = 0

fic_output = np.array(0,dtype=np.uint8)

delta_f = None

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
            
            fft_abs = np.absolute(fft_result)
            
            
            if (x / 2048) < 4:
                 
                if fft_data is None:
                    fft_data = fft_result
                else:
                    fft_data = np.vstack((fft_data, fft_result))
                    
            #window = np.sum(fft_abs[10:fft_len / 2]) + np.sum(fft_abs[fft_len / 2 + 1:10 + carriers + 1])
            
            #window_center = window
            #center = 10 + carriers / 2
            
            #for index in xrange(11, fft_len - 10 - carriers - 1):
            #    window = window - fft_abs[index-1] + fft_abs[index + carriers + 1]
                
            #    if(window > window_center):
            #        center = index + carriers / 2
            #        window_center = window
            
            
            
            #carrier_detect = np.nonzero((np.average(fft_abs) *0.5) < fft_abs)[0]            
            #center = (carrier_detect[0] + carrier_detect[-1]) / 2.
                
            a = np.hstack((fft_abs[:fft_len/2], fft_abs[fft_len/2+1:]))
            b = np.zeros_like(a)

            b[carriers:] = -a[:-carriers]

            center = np.argmax(np.add.accumulate(a + b)) - carriers / 2
            
            if symbol_cnt == 1:
                delta_f = (float(sample_rate) / float(fft_len)) * (center - (fft_len/2 -1))
                
                if abs(delta_f) > 500:
                    correction_factor = (0.5 * correction_factor) + 0.5 * (correction_factor - (delta_f * correction_steps / sample_rate))
                    
                else:
                    correction_factor = (0.99 * correction_factor) + 0.01 * (correction_factor - (delta_f * correction_steps / sample_rate))
                    
            if delta_f is not None and abs(delta_f) < 1000 and symbol_cnt > 1:
                delta_f_fine = sample_rate / fft_len * np.average(np.angle(samples[fft_len:] * np.conjugate(samples[:guard_length]))) / 2. / np.pi
                
                correction_factor = 0.9 * correction_factor + 0.1 * (correction_factor - (correction_steps * delta_f_fine / sample_rate))
            else:
                delta_f_fine = None
                
            correction_factor = int(correction_factor)
            
            if True:
                if symbol_cnt == 1:
                    # phase reference symbol
                    phase_reference = fft_result
                else:
                    # DQPSK demodulation 
                    ofdm_data = fft_result[mapping] * np.conjugate(phase_reference[mapping])
                    phase_reference = fft_result

                    bits = np.hstack((np.real(ofdm_data), np.imag(ofdm_data)))
                    bits = bits/np.max(np.abs(bits))*(-127)
                    
                    
                    #ofdm_data = ofdm_data * 255 / np.abs(ofdm_data)
                    #bits = np.hstack((np.real(ofdm_data), np.imag(ofdm_data)))
                    
                if symbol_cnt == 2:
                    ofdm_fic_data = bits
                elif symbol_cnt > 2 and symbol_cnt <= 4:
                    ofdm_fic_data = np.hstack((ofdm_fic_data, bits))
                    
                if symbol_cnt >= 2:
                    while ofdm_fic_data.size >= 2304:
                        punctured_codeword  = ofdm_fic_data[:2304]
                        ofdm_fic_data       = ofdm_fic_data[2304:]
                        
                        mothercode = fic_puncturer.depuncture(punctured_codeword)

                        from convolutional_encoder import convolutional_encoder
                        from viterby import viterby, euclid_distance
                        conv_encoder = convolutional_encoder((0133,0171,0145,0133), values=(-128,127), dtype=np.int)
                        viterby_decoder = viterby(euclid_distance, conv_encoder)
                        
                        viterby_output = viterby_decoder.decoder(mothercode)
                        output = np.logical_xor(viterby_output, descramble[0:viterby_output.size])
                                
                        output_packed = np.packbits(np.asarray(output,dtype=np.int8))[0:96]

                        fic_output = np.hstack((fic_output, output_packed))

            print (center, delta_f, delta_f_fine, correction_factor, symbol_cnt)
                
        else:
            correction_phase = correction_phase + correction_factor


    offset = offset + 1     


try:
    os.unlink("/tmp/output.dat")
except:
    pass

fic_output.tofile("/tmp/output.dat")
