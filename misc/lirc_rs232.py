#!/usr/bin/python
# -*- coding: utf-8 -*-

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

sio = serial.Serial(args.device,38400,parity=serial.PARITY_EVEN,
                    timeout=0.3)

data = ""
while True:
  value = sio.read(3)

  if value:
    data = data + value
  else:
    data = ""
  
  while len(data) >= 3:
    prefix, time_delta = struct.unpack_from(">BH", data, 0)
    data = data[3:]

    time_delta = time_delta + ((prefix & 0x7F) << 16)
    time_delta = time_delta - 4

    udp_data = int(time_delta * 256 + (15625/2)) / 15625
    
    if udp_data > 0x7FFF:
      udp_data = 0x7FFF

    if prefix & 0x80:
      udp_data = udp_data | 0x8000
      print "space %d" % time_delta
    else:
      pass
      print "pulse %d" % time_delta

    try:
      s.send(struct.pack("<H", udp_data))
    except socket.error:
     pass
      
  
    

