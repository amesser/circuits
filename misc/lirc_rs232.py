#!/usr/bin/python
# -*- coding: utf-8 -*-
# 
# Copyright 2015 Andreas Messer <andi@bastelmap.de>
# 
# RS232 Infrared Receiver to Lirc UDP Converter
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General  *  Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Purpose of this little script is to convert the timing
# information delivered by the receiver hardware
# into a format that lirc can handle

import serial
import struct
import socket
import argparse

parser = argparse.ArgumentParser(description='RS232 to Lirc UDP Converter.')
parser.add_argument('device', type=str, help='Serial Device to use')
parser.add_argument('lirc_host',type=str, help='lirc host to connect to')

args = parser.parse_args()

host,port = args.lirc_host.split(":")
port = int(port)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((host,port))

sio = serial.Serial(args.device,38400,parity=serial.PARITY_EVEN)
sio.timeout=5

data = ""
while True:
  value = sio.read(3)

  if value is None:
    if len(data) < 3:
      data = ""
  else:
    data = data + value
  

  while len(data) >= 3:
    prefix, time_delta = struct.unpack_from(">BH", data, 0)
    data = data[3:]

    time_delta = time_delta + ((prefix & 0x7F) << 16)
    time_delta = time_delta - 4

    udp_data = int(time_delta * 256 + (15625/2)) / 15625
    
    if udp_data > 0x7FFF:
      udp_data = 0x7FFF

    if prefix & 0x80:
      # print "space %d" % time_delta
      udp_data = udp_data | 0x8000
    else:
      #print "pulse %d" % time_delta
      pass

    try:
      s.send(struct.pack("<H", udp_data))
    except socket.error:
     pass
      
  
    

