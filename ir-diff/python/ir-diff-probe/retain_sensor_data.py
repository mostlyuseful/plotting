#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pathlib
import click
import serial
import io
from time import time


def collect(ser: serial.Serial, outfile):
    while True:
        line = ser.readline().strip()
        t = time()
        outfile.write('{time};{line}\n'.format(time=t,line=line.decode()))
        outfile.flush()
        print('#',end='',flush=True)


@click.command()
@click.argument('devpath', type=click.Path())
@click.argument('logpath', type=click.Path(dir_okay=False, writable=True))
def main(devpath, logpath):
    ser = serial.Serial(devpath, baudrate=115200)
    with io.open(logpath, 'w', encoding='utf-8') as f:
        collect(ser, f)


if __name__ == '__main__':
    main()
