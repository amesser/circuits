#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Plot part selection curves for multiple feedback amplifiers
# Copyright (C) 2016 Andreas Messer <andi@bastelmap.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy    as np
import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerNpoints, HandlerLine2D

a1 = 1.3397
b1 = 0.4889
a2 = 0.7743
b2 = 0.3890
 
def func_a(a,b,c1c2,A0):
    """Return R2*fc*C1 as function of a,b C1/C2 and A0"""
    
    x = a*a - 4*b * c1c2 * (1-A0)
    x = np.ma.masked_less(x, 0.0, False)
    x = a - np.sqrt(x)
    x = x / (4.0 * np.pi)

    return x

def func_b(a,b,c1c2,A0):
    """Return R3*fc*C2 as function of a,b C1/C2 and A0"""
    x = func_a(a,b,c1c2,A0)
    x = b / (4.0 * np.pi * np.pi * x)

    return x

def func_r1(a,b, fc, c1, c2,A0):
    x = func_r2(a,b,fc,c1,c2,A0)
    x = - x / A0
    return x

def func_r2(a,b, fc, c1, c2,A0):
    x = func_a(a,b,c1/c2,A0)
    x = x / (c1 * fc)
    return x

def func_r3(a,b, fc, c1, c2,A0):
    x = func_b(a,b,c1/c2,A0)
    x = x / (c2 * fc)
    return x


capacitors = np.asarray([1.0, 1.5, 2.2, 3.3, 4.7, 6.8], dtype = np.double)
resistors  = np.asarray([1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2], dtype = np.double)

c1_values = np.asarray([100e-12, 150e-12, 220e-12, 330e-12, 470e-12,680e-12, 1e-9])

c2_powers = np.logspace(-10, -7, 4)
c2_values = np.tile(capacitors, c2_powers.size) * np.repeat(c2_powers,capacitors.size)
 
 
def generate_resistorplots(a,b, fc, A0):
    plt.figure()
    color_legend_handles = []
    color_legend_labels  = []

    symbol_legend_handles = []
    
    
    colors = ['b','g','r','c','m','y','k','w']
    for c1 in c1_values:
        indices = np.nonzero(c2_values > (c1 * (4*b*(1-A0)) / (a*a)))
        
        xvalues = c2_values[indices]
        
        yvalues_r1 = func_r1(a,b,fc,c1, xvalues, -100)
        yvalues_r2 = func_r2(a,b,fc,c1, xvalues, -100)
        yvalues_r3 = func_r3(a,b,fc,c1, xvalues, -100)
 
        if(np.any(yvalues_r1 > 100) and
           np.any(yvalues_r2 < 100e3)):
            
            c = colors.pop(0)
            
            p1, = plt.loglog(xvalues, yvalues_r1, color = c, marker='o',label="R1")
            p2, = plt.loglog(xvalues, yvalues_r2, color = c, marker='^',label="R2")
            p3, = plt.loglog(xvalues, yvalues_r3, color = c, marker='s',label="R3")

            color_legend_handles.append(p1)
            color_legend_labels.append('C1=%eF' % c1)

            symbol_legend_handles = [p1,p2,p3]

    r_powers = np.logspace(1, 4, 4)
    r_values = np.tile(resistors, r_powers.size) * np.repeat(r_powers,resistors.size)
    
    ax = plt.axes()
    ax.set_yticks([100, 1e3, 10e3, 100e3], minor=False)
    ax.set_yticks(r_values, minor=True)

    ax.yaxis.grid(True, linestyle='-', which='major')
    ax.yaxis.grid(True, which='minor')

    plt.xlabel('C2/F')
    plt.ylabel('R/Ohm')
    plt.title('fc=%.0f, A0=%.0f, a=%.4f, b=%.4f' % (fc,A0,a,b))
    
    l = plt.legend(color_legend_handles, color_legend_labels, numpoints = 1, loc=1)
    ax = plt.gca().add_artist(l)
    
    plt.legend(handles=symbol_legend_handles, numpoints = 1, loc=2)
 
generate_resistorplots(a1,b1,11000,-100)
generate_resistorplots(a2,b2,11000,-100)
plt.show()
