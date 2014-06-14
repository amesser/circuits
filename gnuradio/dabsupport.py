#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import cmath
import sys

num_carriers = {
  'I' : 1536, 
}

num_frequencies = {
  'I' : 2048, 
}

perm_filter = {
  'I' : (lambda x : x >= 256 and x <= 1792 and x != 1024), 
}

phase_n = {
  'I' : (1,2,0,1,3,2,2,3,2,1,2,3,1,2,3,3,2,2,2,1,1,3,1,2,3,1,1,1,2,2,1,0,2,2,3,3,0,2,1,3,3,3,3,0,3,0,1,1),
}


def frequency_interleaving(mode):
    global num_carriers, num_frequencies, perm_filter
    
    permutation = [0]
    
    while len(permutation) < num_frequencies[mode]:
        permutation.append((13 * permutation[-1] + 511) % num_frequencies[mode])
        
    i = [x for x in range(num_frequencies[mode]) if perm_filter[mode](permutation[x])]
    D = [permutation[x] for x in i]

    if len(D) != num_carriers[mode]:
        raise ValueError()
    
    n = list(range(num_carriers[mode]))
    
    k = [D[x] - 1024 for x in n]
    
    for x in zip(i,D,n,k):
        print( "%5d %5d %5d %5d" % x)
        
    return dict(zip(i,k))
        
def phase_symbol(mode):
    k_min = list(range(-768, -1, 32)) + list(range(1, 768, 32))
    
        
    def ks(k):
        return list(k_min[x] for x in range(len(k_min)) if k >= k_min[x])[-1]
    
    def i(k):
        if k < 0:
            return int(((ks(k) + 768) / 32) % 4)
        else:
            return int(((769 - ks(k)) / 32) % 4)
                
    def h(i,j):
        return (
          0,2,0,0,0,0,1,1,2,0,0,0,2,2,1,1,0,2,0,0,0,0,1,1,2,0,0,0,2,2,1,1,
          0,3,2,3,0,1,3,0,2,1,2,3,2,3,3,0,0,3,2,3,0,1,3,0,2,1,2,3,2,3,3,0,
          0,0,0,2,0,2,1,3,2,2,0,2,2,0,1,3,0,0,0,2,0,2,1,3,2,2,0,2,2,0,1,3,
          0,1,2,1,0,3,3,2,2,3,2,1,2,1,3,2,0,1,2,1,0,3,3,2,2,3,2,1,2,1,3,2
        )[j + 32 * i]
        
    def n(k):
        if k < 0:
            return phase_n[mode][int((768 + k) / 32)]
        else:
            return phase_n[mode][int((768 + k - 1) / 32)]
        
    def phi(k):
        return cmath.pi * (h(i(k),k-ks(k)) + n(k)) / 2.
    
    def z(k):
        if k == 0:
            return cmath.rect(0,0)
        else:
            return cmath.exp(cmath.rect(1, phi(k)))
        
    for k in list(range(-768,-1)) + list(range(1,768)):
        print("%5d %s" %(k, str(z(k))))
        
    return dict( (k,z(k)) for k in list(range(-768,-1)) + list(range(1,768)))
    
class dab(object):
  @property
  def z1(self):
    i_to_k = frequency_interleaving('I')
    k_to_z1 = phase_symbol('I')

    symbol2 = []
    for x in range(2048):
        try:
            symbol2.append(k_to_z1[i_to_k[x]])
        except KeyError:
            symbol2.append(cmath.rect(0,0))

    return symbol2
    
if __name__ == "__main__":
    i_to_k = frequency_interleaving('I')
    
    print("============")
    
    k_to_z1 = phase_symbol('I')
    
    print("============")
    
    for x in range(2048):
        try:
            print("%5d %s" % (x, k_to_z1[i_to_k[x]]))
        except KeyError:
            print("%5d %s" % (x, cmath.rect(0,0)))

    print("============")
    
    symbol2 = []
    for x in range(2048):
        try:
            symbol2.append(k_to_z1[i_to_k[x]])
        except KeyError:
            symbol2.append(cmath.rect(0,0))
            
    print (symbol2)

    
    
    
