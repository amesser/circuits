#!/usr/bin/python
# -*- coding: utf-8 -*-

import struct
import argparse

def dump_minmax(prefix, data):
    max, min = struct.unpack("<2H", data)
    print("{0}: Maximum {1}, Minimum {2}".format(prefix,max,min))

def dump_linreg(prefix, data):
    sumx, sumy, sumxy, sumxx = struct.unpack("<4L", data[0:16])

    print("{0}: SumX  {1}".format(prefix,sumx))
    print("{0}: SumY  {1}".format(prefix,sumy))
    print("{0}: SumXY {1}".format(prefix,sumxy))
    print("{0}: SumXX {1}".format(prefix,sumxx))

    NumPoints = struct.unpack("<25B", data[16:])
    StrPoints = ("{0:3}".format(x) for x in NumPoints)
    StrTemp   = ("{0:3}".format(t) for t in range(-10,40,2))

    print("{0}: NumPoints   T   C".format(prefix))
    for t,c in zip(StrTemp,StrPoints):
        print("{0}            {1} {2}".format(" " * len(prefix), t, c))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dump calibrator eeprom content.")
    parser.add_argument("file", type=argparse.FileType("rb"))

    args = parser.parse_args()
    data = args.file.read(4 + 4 + 16 + 25)

    dump_minmax("Humidity", data[0:4])
    dump_minmax("Light", data[4:8])
    dump_linreg("Temp", data[8:])

