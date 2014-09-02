#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class puncturing(object):
    def __init__(self):
        self.mask = np.asarray([[1,1,0,0],[1,0,0,0]])
        
    def puncture(self, input):
        mask  = self.mask.flatten()
        input = input.flatten()
        
        selection = np.tile(mask, input.size / mask.size)
        
        return input[np.nonzero(selection)]

    def depuncture(self, input):
        mask      = self.mask.flatten()
        numoutput = input.size * mask.size / np.count_nonzero(mask)
        
        output = np.ma.masked_all(numoutput, dtype=input.dtype)
        selection = np.tile(mask, output.size / mask.size)
        
        output[np.nonzero(selection)] = input
        
        output = output.reshape( (-1,) + self.mask.shape[1:])
        
        return output
        
if __name__ == "__main__":
    from convolutional_encoder import convolutional_encoder as conv_enc
    from viterby import viterby, hamming_distance
    test_cv_encoder = conv_enc((0133,0171,0145,0133))
    #test_cv_encoder = conv_enc((07,05))
            
    data = np.unpackbits(np.arange(256, dtype=np.uint8))
    
    
    cv_data = np.zeros(shape=data.shape + (4,), dtype=np.uint8)
    
    for i in xrange(data.size):
        cv_data[i] =  test_cv_encoder.getOutput(data[i])
        
    test_puncturer = puncturing()

    punctured_data = test_puncturer.puncture(cv_data)

    depunctured_data = test_puncturer.depuncture(punctured_data)
            
    test_viterby = viterby(hamming_distance, test_cv_encoder)
    
    if ((test_viterby.decoder(depunctured_data) != data).any()):
        raise Exception("blbu")
    
        