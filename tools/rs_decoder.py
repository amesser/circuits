#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

multiply_table = np.zeros((256,256), dtype=np.uint8)

for a in xrange(multiply_table.shape[0]):
    for b in xrange(multiply_table.shape[1]):
        value      = 0
        
        for i in xrange(8):
            if b & (0x1 << i):
                value = value ^ (a << i)
                
        mask_poly  = 0b100011101 << 8
        mask_bit   = 0x10000
        
        while value >= 256:
            if mask_bit & value:
                value = value ^ mask_poly
                
            mask_bit  = mask_bit  >> 1
            mask_poly = mask_poly >> 1
        
        multiply_table[a,b] = value
        
print(multiply_table)

divide_table = np.zeros_like(multiply_table)
        
for a in xrange(multiply_table.shape[0]):
    for b in xrange(multiply_table.shape[1]):
        # a * b = result 
        # -> a = result / b
        # -> b = result / a
        result = multiply_table[a][b]
        
        if b:
            divide_table[result,b] = a
        if a:
            divide_table[result,a] = b
        
def coefficient(i):
    result = 1
    while i > 0:
        result = multiply_table[result,2]
        i = i -1
    return result

def generate_poly(n):
    poly = np.zeros((n+1), dtype=np.uint8)
    poly[0] = 1
    
    for i in xrange(n):
        alpha = coefficient(i)
        
        # multiply poly by x
        poly_a = np.zeros_like(poly)
        poly_a [1:] = poly[0:-1]

        # multiply poly by constant
        multiply_slice = multiply_table[alpha]
        
        poly_b = multiply_slice[poly]
        
        poly = poly_a ^ poly_b
    return poly


def multiply_poly(a,b):
    # (x^0, ... ,x^(i-3), x^(i-2), x^(i-1) ,x^i) -> a
    #                   ( x^0,     x^(j-1) ,x^j) -> b
    result = np.zeros((a.size+b.size-1), dtype=a.dtype)
    for i in xrange(a.size):
        multiply_slice = multiply_table[a[i]]
        result[i:i+b.size] = result[i:i+b.size] ^ multiply_slice[b]
        
    return result

def eval_poly(p, x):
    power_x  = p.dtype.type(1)
    sum_poly = p.dtype.type(0)

    for i in xrange(0, p.size):
        sum_poly = sum_poly ^ multiply_table[power_x, p[i]]
        power_x = multiply_table[power_x,x]

    return sum_poly

start_poly = np.ones(1, dtype=np.uint8)

for i in xrange(10):
    print (generate_poly(i))
    print (start_poly)
    start_poly = multiply_poly(start_poly, np.asarray([coefficient(i),1],dtype=np.uint8))
    
print multiply_table.size

def checksum(input, poly):
    # ( x^i, x^(i-1),.... x^0) -> (x^0, ..., x^(i-1), x^i) 
    input = np.copy(input)[::-1]
    
    for offset in xrange(input.size - poly.size, -1, -1):
        # (x^0, ... ,x^(i-3), x^(i-2), x^(i-1) ,x^i) -> input
        #                   ( x^0,     x^(j-1) ,x^j) -> poly
        # ---------------------------------------------------
        # (x^0, ... ,x^(i-3), x^(i-2), x^(i-1) , 0) -> input
        #          ( x^0,     x^(j-1) ,x^j)          -> poly
        # .
        # .
        #.
        
        # get the highest power of the current poly divsiion stage
        f = input[offset + poly.size - 1]
        
        # multiply poly with f
        multiply_slice = multiply_table[f]
        poly_add = multiply_slice[poly]
        
        # add poly to input
        input[offset: offset + poly.size] = \
          input[offset: offset + poly.size] ^ poly_add
          
    return input[0:poly.size] 
        
def find_errors(input, synd):
    error_poly    = np.zeros_like(synd)
    error_poly[0] = 1
    
    old_poly = np.copy(error_poly)
    
    error_poly_length = 1
    old_poly_length   = 1
    
    for i in xrange(synd.size):
        old_poly[1:] = old_poly[0:-1]
        old_poly[0]  = 0
        old_poly_length += 1
        
        delta = synd[i]
        
        for j in xrange(1, error_poly_length):
            delta = delta ^ multiply_table[error_poly[j], synd[i-j]]
            
        if delta != 0:
            if old_poly_length > error_poly_length:
                multiply_slice = multiply_table[delta]
                
                new_poly        = multiply_slice[old_poly]
                new_poly_length = old_poly_length 

                multiply_slice = multiply_table[divide_table[1,delta]]
                
                old_poly = multiply_slice[error_poly]
                old_poly_length = error_poly_length
                
                error_poly = new_poly
                error_poly_length = new_poly_length

            multiply_slice = multiply_table[delta]

            error_poly = error_poly ^ multiply_slice[old_poly]
            #error_poly_length = 1 + np.max(np.nonzero(error_poly))
            
    error_poly = error_poly[0:error_poly_length]
    errors = error_poly_length - 1
    
    if errors * 2 > synd.size:
        return None
    
    error_poly_result = np.zeros(255, dtype = input.dtype)
    
    power_x = np.uint8(1)
    for i in xrange(0, 255):
        power_x = multiply_table[2,power_x]
        error_poly_result[i] = eval_poly(error_poly, power_x)
                
    error_poly_result = error_poly_result[255 - input.size:]
    return np.nonzero(np.zeros_like(error_poly_result) == error_poly_result)[0]
    
def correct_errors(message, synd, pos):
    q = np.zeros(1, dtype=np.uint8)
    q[0] = 1

    multiply_slice = multiply_table[2]
    for i in range(pos.size):
        x = np.uint8(1)
        
        for j in range(message.size - 1 - pos[i]):
            x = multiply_slice[x]
            
        q = multiply_poly(q, np.asarray([1,x], dtype=np.uint8))
        
    p = synd[:pos.size]
    p = multiply_poly(q,p)
    p = p[:pos.size]
    
    qprime = q[1::2]

    for i in range(pos.size):
        x = np.uint8(1)
        
        for j in range(pos[i] + 256 - message.size):
            x = multiply_slice[x]
            
        y = eval_poly(p, x)
        z = eval_poly(qprime, multiply_table[x,x])
        
        message[pos[i]] = message[pos[i]] ^ divide_table[y, multiply_table[x,z]] 
    