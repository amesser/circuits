#! /usr/bin/env python
# encoding: utf-8
#
# Tool to read out and examine images generated by the sdlogger
#
# Copyright 2013 Andreas Messer <andi@bastelmap.de>
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

import sys
import struct

import numpy as np
import argparse


class SDLoggerPDU(object):
    blocktypes = 'invalid config data calibration'.split()
    channelids = 'ch0 ch1 ch2 ch3 ch0-ch1 ch1-ch0 ch2-ch3 ch3-ch2 di'.split()
    
    @classmethod
    def getBlockId(cls, name):
        return cls.blocktypes.index(name)

    @classmethod
    def getChannelId(cls, name):
        return cls.channelids.index(name)
    
    def writeConfigPdu(self,file):
        fields = '<2B H %uB B' % len(self.channels)
        values = (self.getBlockId('config'), struct.calcsize(fields) - 3, self.interval) \
                 + tuple(map(self.getChannelId, self.channels)) \
                 + (0xFF,)
                 
        return file.write(struct.pack(fields,*values))

class Container(): pass

        
        



def readConfig(file):
    container = Container()
    
    container.type,container.length = \
        struct.unpack("<2B",file.read(2))
    
    if (container.type != 1):
        raise Exception('Invalid block type %u' % container.type)
        
    nchan = (container.length - 2) / 1
          
    x = struct.unpack("<H %uB x" % nchan, file.read(container.length + 1))
    
    container.interval = x[0]
    container.channels = x[1:]
    
    return container

def readCalibration(file):
    container = Container()
    
    container.type,container.length = \
        struct.unpack("<2B",file.read(2))
    
    if (container.type != 3):
        raise Exception('Invalid block type %u' % container.type)
        
          
    container.reference, = struct.unpack("<H x", file.read(container.length + 1))
    
    return container

def readData(file, channels, blocksize):
    container = DataBlockFactory(channels, blocksize)
    
    return arr

class Dumper(object):
    def dump(self, file,channels):
      config = readConfig(file)
       
      self.dumpConfig(config)
                  
      file.seek(512)
      
      calibration = readCalibration(file)
      
      self.dumpCalibration(calibration)
            
      nchan = len(config.channels)      
      
      dtype = '<u1,<u1,<u4,<%uu2,<u1' % nchan
      shape = (512,)

      done = False
      
      selection = channels or range(nchan)
      
      while not done:
          arr = np.ndarray(shape=shape,dtype=dtype)
          
          file.readinto(arr)
          
          container = Container()

          done = np.any(arr['f0'] != 2)
          
          
          if (done):
              valid = np.nonzero(arr['f0'] == 2)[0]
              container.ts   = arr['f2'][valid]
              container.data = [arr['f3'][valid,x] & 0xFFF for x in selection]
          else:    
              container.ts   = arr['f2']
              container.data = [arr['f3'][:,x] & 0xFFF for x in selection]
                      
          self.dumpData(container)
          
      self.dumpDone()    
                

class StdoutDumper(Dumper):
    count = 0
    
    def dumpConfig(self,config):
        sys.stdout.write('# Interval: %ums, ' % config.interval +
                         ', '.join(['channel %u' % x for x in config.channels])+
                         '\n')
        
    def dumpCalibration(self,calibration):
        self.reference = float(calibration.reference) / 1000.    
        sys.stdout.write('# Calibration: %fV\n' % self.reference)
        
    def dumpData(self,data):
        arr = np.zeros(shape=(len(data.ts),),dtype=('u4' + ',d' * len(data.data)))
            
        arr['f0'] = data.ts
        
        for x in range(len(data.data)):
            arr['f%u' % (x+1)] = data.data[x] * self.reference / 4096.    

        np.savetxt(sys.stdout.buffer, arr, fmt=("%u" + " %f" * len(data.data)))
        
        # for x in np.nditer([data.ts] + data.data):
        #    sys.stdout.write('%u ' % x[0] + ' '.join(map(lambda x : '%f' % (self.reference * x / 4096.) ,x[1:])) + '\n')
            
        sys.stderr.write('Read %u records\n' % self.count)
        
    def dumpDone(self):
        pass        

class PlotDumper(Dumper):
    count = 0
    
    def dumpConfig(self,config):
        self.nchan = len(config.channels)
        
        sys.stdout.write('# Interval: %ums, ' % config.interval +
                         ', '.join(['channel %u' % x for x in config.channels])+
                         '\n')
        
    def dumpCalibration(self,calibration):
        self.reference = float(calibration.reference) / 1000.    
        
    def dumpData(self,data):
        data.data = list((self.reference * x / 4096.) for x in data.data)
        try:
            self.block.append(data)
        except AttributeError:
            self.block = [data] 
            
        self.count = self.count + data.ts.shape[0]       
        
        sys.stderr.write('Read %u records\n' % self.count)    

    def dumpDone(self):
        ts = np.hstack(x.ts for x in self.block)
        
        nchan = len(self.block[0].data)               
        data = [np.hstack(x.data[y] for x in self.block) for y in range(nchan)]

        del self.block
        
        formats = ['r,','g,','b,','y,']

        flat = sum(((ts, data[x], formats[x]) for x in range(nchan)),tuple())

        import matplotlib.pyplot as plt
      
        plt.plot(*flat)
        plt.show()

def dump(file, channel):
  StdoutDumper().dump(file,channel)

def plot(file, channel):
  PlotDumper().dump(file,channel)
  
def config(file, interval, channels):
    file.seek(0,2)
    filesize = file.tell()
    file.seek(0,0)

    sys.stdout.write('Card Size is %0.1f MB.\n' % (filesize/1024./1024.))
    
    pdu = SDLoggerPDU()
    pdu.interval = interval
    pdu.channels = channels
    
    sys.stdout.write('Writing Configuration...')
    blocksize = 4096
    len = pdu.writeConfigPdu(file)
    
    empty = b'\xff' * blocksize
    
    len += file.write(empty[0:blocksize - len])

    sys.stdout.write(' done.\n')
    
    sys.stdout.write('Initializing Card... ')
    sys.stdout.flush()
    
    progress = 0
    while len < filesize:
      len += file.write(empty[0:min(blocksize,filesize-len)])
      if int(10 * len / filesize) > progress:
        progress = int(10 * len / filesize)
        sys.stdout.write('#')
        sys.stdout.flush()

    file.close()
      
    sys.stdout.write(' done.\n')
             
def main():
  parser = argparse.ArgumentParser(description='SDLogger Image File Tool')
  subparsers = parser.add_subparsers(help='sub-command help')
  
  p = subparsers.add_parser('dump', help='dump records')
  p.add_argument('file', type=argparse.FileType('rb'))
  p.add_argument('--channel', '-c', type=int, action='append')
  p.set_defaults(func=dump)
  
  p = subparsers.add_parser('plot', help='plot records')
  p.add_argument('file', type=argparse.FileType('rb'))
  p.add_argument('--channel', '-c', type=int, action='append')
  p.set_defaults(func=plot)
  
  p = subparsers.add_parser('config', help='generate configuration')
  p.add_argument('file', type=argparse.FileType('wb'))
  p.add_argument('interval', type=int, help='sample interval in ms. (1 to 60000)')
  p.add_argument('channels', metavar='channel', type=str, nargs='+', help='channel to sample. (%s)' % ', '. join(SDLoggerPDU.channelids))
  p.set_defaults(func=config)
  
  args   = parser.parse_args()
  kwargs = dict(vars(args))
  del kwargs['func']

  args.func(**kwargs)
  
if __name__ == '__main__':
  main()
