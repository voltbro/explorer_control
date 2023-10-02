#!/usr/bin/env python3

import sys

import hid

device = hid.device()

device.open(0x483, 0x5710)

size = 23 if len(sys.argv) < 2 else int(sys.argv[1])
mode = None if len(sys.argv) < 3 else sys.argv[2]
j = 5

while True:
    data = device.read(size, 100)
    if not data:
       continue
    if mode == 'sum':
        data = [data[0] + data[1], data[2] + data[3]]
    if mode == 'proc':
        dt = []
        dt.append(data[0] | (data[1] << 8))
        dt.append(data[2] | (data[3] << 8))
        data = dt
        j = 15
    print(''.join(str(v).ljust(j) for v in data))
