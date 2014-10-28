#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from puncturing import puncturing
from convolutional_encoder import convolutional_encoder
from viterby import viterby, euclid_distance

puncturing_vectors = np.zeros(shape=(24,32), dtype=np.bool8)
                           
for pi in xrange(1,25):
    puncturing_mask = puncturing_vectors[pi-1]
    
    puncturing_mask[::4] = True
    
    offset   =[1,  1+8, 1+4, 1+12, 2, 2+8, 2+4, 2+12, 3, 3+8, 3+4, 3+12]
    
    while pi:
        for x in xrange(offset[0],32,16):
            if pi and not puncturing_mask[x]:
                puncturing_mask[x] = True
                pi -= 1
        
        offset.pop(0)
        
puncturing_remainder = np.zeros(shape=24, dtype=np.bool8)
puncturing_remainder[0::4] = True
puncturing_remainder[1::4] = True

def scrambler(length):
    value = 0x1FF;
    
    for x in range(length):
        result = 0 if (0x110 & value) in (0, 0x110) else 1
        value = value << 1 | result
        yield result
         
descramble = np.fromiter(scrambler(8192),dtype=np.bool8, count=8192)


class msc_decoder(object):
    def __init__(self, db):
        self.db       = db
        self.cnt      = 0
        self.msc_list = []
        self.output_file = open("/tmp/subchannel.dat", "wb")
        
    def phase_symbol(self, sym):
        self.cnt = 1
        self.msc_assemble = None

    bitreversal = (0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15)

    def data_symbol(self, sym):
        self.cnt += 1
        msc_deinterleave = None
        
        if self.cnt > 4 and self.cnt <= 76:
            
            if self.msc_assemble is not None:
                self.msc_assemble = np.hstack((self.msc_assemble, sym))
            else:
                self.msc_assemble = sym
                
            if self.msc_assemble.size >= 55296:
                self.msc_list.append(self.msc_assemble)
                self.msc_assemble = None 
                
                if len(self.msc_list) >= 16:
                    msc_deinterleave = np.zeros_like(self.msc_list[0])
                    
                    for start in xrange(0,16):
                        deltat = self.bitreversal[start]
                        msc_deinterleave[start::16] = self.msc_list[deltat][start::16]
                        
                    self.msc_list.pop(0)
        
        if msc_deinterleave is not None:
            self.decode_msc(msc_deinterleave)
            
    cu_size = 64
    
    eep_protection_table = {
      # 3-A
      (0, 2) : ( 6, ( (lambda n : 6*n-3), (lambda n: 3)),(8, 7)),
    }
    
    def decode_msc(self, msc):
        subchannel = self.db.audio_channel_to_decode
        cu_size = self.cu_size
        
        if subchannel:
            start, size = subchannel.startaddress, subchannel.size
            subchannel_data = msc[(cu_size * start):(cu_size * (start+size))]
            
            print ("extracted {0} bits msc data".format(subchannel_data.size))
            
            index = (subchannel.option, subchannel.protection_level)
            
            size_cu, L,P = self.eep_protection_table[index]
            
            n = size / size_cu
            
            puncturer = puncturing()
            puncturer.mask = np.hstack((\
                               np.tile(puncturing_vectors[P[0]-1], 4* L[0](n)),\
                               np.tile(puncturing_vectors[P[1]-1], 4* L[1](n)),\
                               puncturing_remainder,\
                             ))
            puncturer.mask = puncturer.mask.reshape((-1,4))

            print ("created {0} bits puncture mask".format(np.count_nonzero(puncturer.mask)))

            mothercode = puncturer.depuncture(subchannel_data)
            
            print ("extracted {0} bits msc data mothercode".format(mothercode.size))
            
            conv_encoder = convolutional_encoder((0133,0171,0145,0133), values=(-128,127), dtype=np.int)
            viterby_decoder = viterby(euclid_distance, conv_encoder)
            
            viterby_output = viterby_decoder.decoder(mothercode)[0:192 * n]
            output = np.logical_xor(viterby_output, descramble[0:viterby_output.size])
            
            output_packed = np.packbits(np.asarray(output,dtype=np.uint8))
            output_packed = output_packed.tostring()
            
            self.output_file.write(output_packed)


if __name__ == "__main__":
    for pi in xrange(1,25):
        print ("pi {0}".format(pi) + str (puncturing_vectors[pi-1]))
