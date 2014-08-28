#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

def hamming_distance(a,b):
    #print("{0}\t{1}".format(a,b))
    return np.sum(a != b, axis=(-1))

class viterby(object):
    
    def __init__(self, distance, encoder):
        self.distance = distance
        self.encoder  = encoder
        
    def decoder(self, input_data):
        symbol_count, symbol_length = input_data.shape
        state_count = len(self.encoder.states)
        
        accumulated_metric_table = np.zeros((symbol_count+1,state_count), dtype = np.uint) 
        state_history_table      = np.zeros_like(accumulated_metric_table)
        
        for i in xrange(symbol_count):
            states = self.encoder.states
            
            prev_states  = self.encoder.prevStates(states)
            output = self.encoder.prevOutputs(states)

            dist = self.distance(output,input_data[i])
            dist = dist + accumulated_metric_table[i][prev_states]

            cond = dist[:,0] < dist[:,1]
            
            state_history_table[i+1,:]       = np.where(cond, prev_states[:,0],prev_states[:,1])
            accumulated_metric_table[i+1][:] = np.where(cond, dist[:,0],       dist[:,1])
            
            #for j in xrange(state_count):
            #    a,b = self.encoder.prevStates(j)
            #    
            #    out_a, out_b = self.encoder.prevOutputs(j)
            #    
            #    dist_a = self.distance(out_a,input_data[i])
            #    dist_b = self.distance(out_b,input_data[i])
            #    
            #    dist_a += accumulated_metric_table[i][a]
            #    dist_b += accumulated_metric_table[i][b]
            #    
            #    if dist_a > dist_b:
            #        state_history_table[i+1][j] = b
            #        accumulated_metric_table[i+1][j] = dist_b
            #    else:
            #        state_history_table[i+1][j] = a
            #        accumulated_metric_table[i+1][j] = dist_a
        
        survivor_state_table = np.zeros(symbol_count+1,dtype = np.uint)
        output_table = np.zeros(symbol_count)
        survivor_state_table[-1] = np.argmin(accumulated_metric_table[-1])
        for i in xrange(symbol_count, 0, -1):
            survivor_state_table[i-1] = state_history_table[i][survivor_state_table[i]]
            output_table[i-1] = self.encoder.prevInput(survivor_state_table[i])
        
        return output_table

if __name__ == "__main__":
    from convolutional_encoder import convolutional_encoder as conv_enc
    test_cv_encoder = conv_enc((0133,0171,0145,0133))
    #test_cv_encoder = conv_enc((07,05))
            
    data = 0b100010100111010
    
    test_data = np.zeros((32,4),dtype = np.bool8)
    for i in range(32):
        test_data[i] =  test_cv_encoder.getOutput(data & 0x1)
        data = data >> 1
    
    test_viterby = viterby(hamming_distance, test_cv_encoder)
    
    print("real output: {0}".format(test_data))
    print("real input: " + "".join(reversed("{0:032b}".format(0b100010100111010))))
    print("calc input: {0}".format("".join(str(int(x)) for x in test_viterby.decoder(test_data))))