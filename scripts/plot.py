#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

dtype = np.dtype('u1,3u1,u1,u1,3u2,u2,u2')
dtype = dtype.newbyteorder('>')

data = np.fromfile("test.dat",dtype=dtype,count=-1)

# take all records where the both copies match each other
indices = np.nonzero(data[0::2] == data[1::2])
data    = data[0::2][indices]

# sanity check
indices = np.nonzero((data['f0'] == 0) | \
                     (data['f2'] == 192) | \
                     (data['f3'] == 0))

data = data[indices]

fmt = '{0:02}:{1:02}:{2:02} {3:5} {4:5} {5:5} {6:4}'.format
for x in data:
  (second,minute,hour) = x[1]
  (humidity, light, temp) = x[4]
  (BRPR) = x[6]

  print (fmt(hour,minute,second,humidity, light,temp,BRPR))

time = np.asarray(data['f1'],dtype=np.dtype('u4'))
time = time[:,0] + time[:,1] * 60 + time[:,2] * 60 * 60

humidity = data['f4'][:,0]
light = data['f4'][:,1]
temp = data['f4'][:,2]
BRPR = data['f6']

lh, = plt.plot(time, humidity, 'r-', label='Humidity')
ll, = plt.plot(time, light,    'g-', label='Light')
lt, = plt.plot(time, temp,     'b-', label='Temp')

plt.legend([lh,ll,lt],['Humidity', 'Light', 'Temp'])
plt.xlabel('Time')
plt.ylim(0,2000)


plt.show()
