#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class convolutional_encoder(object):
    def __init__(self, polynoms):
        polynoms = np.asarray(polynoms, dtype=np.uint)
        
        self.state = 0
        self.grade = 0
        
        x = int(np.max(polynoms))
        
        while x:
            self.grade += 1
            x = x / 2
            
        state_enumeration = np.arange(2**(self.grade-1), dtype=np.uint)
        
        parity_table = np.zeros_like(state_enumeration, dtype=np.bool8)
        
        helper = np.array(state_enumeration)
        
        for i in range(self.grade):
            parity_table = np.logical_xor(parity_table, (helper != 0))
            helper = (helper - 1) & helper

        a = np.repeat(state_enumeration, polynoms.size).reshape(state_enumeration.size, polynoms.size)
        b = np.tile(polynoms, state_enumeration.size).reshape(a.shape)
        
        
        output_flip  = np.asarray(polynoms & (1 << (self.grade-1)), dtype=np.bool8)
                
        c = parity_table[a & b]
        
        c = np.repeat(c,2,axis=0).reshape(state_enumeration.size, 2, polynoms.size)
        c[:,1] ^= output_flip
        
        self.output_table = c
        
        helper = np.array(state_enumeration)
        
        helper = np.repeat(helper,2) / 2
        helper[1::2] += 2**(self.grade - 2)

        self.nextstate_table = helper.reshape(state_enumeration.size, 2)
        
        self.prevstate_table = np.tile(state_enumeration,2).reshape(-1,2)
        
    
    def getOutput(self, bit):
        if bit:
            bit = 1
        else:
            bit = 0
            
        state = self.state

        self.state = int(self.nextstate_table[state, bit])
        return self.output_table[state,bit]
    
    def prevStates(self,state):
        return self.prevstate_table[state]  

    def prevOutputs(self,state):
        bit = (state << 1) & 2**(self.grade - 1)
        
        return self.output_table[self.prevStates(state),bit]
    
if __name__ == "__main__": 
    dab_cv_encoder = convolutional_encoder((0133,0171,0145,0133))
            
    data = 0x005612AF
    
    for i in range(32):
        print("{0:2d} {1} {2:06b} {3:06b} {4:06b} {5:06b} {6:06b} {7}".format(i, data & 0x1,\
            dab_cv_encoder.state & 0133,\
            dab_cv_encoder.state & 0171,\
            dab_cv_encoder.state & 0145,\
            dab_cv_encoder.state & 0133,\
            dab_cv_encoder.state, dab_cv_encoder.getOutput(data & 0x1)))
        data = data >> 1
        
    print(dab_cv_encoder.prevstate_table)
    print(dab_cv_encoder.prevOutputs(0))
    
    test_cv_encoder = convolutional_encoder((07,05))
            
    data = 0b100010100111010
    
    for i in range(32):
        print("{0:2d} {1} {2:02b} {3:02b} {4:02b} {5}".format(i, data & 0x1,\
            test_cv_encoder.state & 07,\
            test_cv_encoder.state & 05,\
            test_cv_encoder.state, test_cv_encoder.getOutput(data & 0x1)))
        data = data >> 1        