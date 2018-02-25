#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import click
import serial

def cmd(ser, command):
    if type(command) != bytes:
        command = command.encode()
    ser.write(command)
    ser.write(b'\r\n')
    return ser.readline()


@click.command()
@click.option('--zstart', type=float, default=11.0)
@click.option('--zend', type=float, default=100.0)
@click.option('--home/--no-home', default=False)
@click.option('--x', type=float, default=None)
@click.option('--y', type=float, default=None)
@click.argument('ctlport', type=click.Path())
@click.argument('sensorport', type=click.Path())
def main(zstart, zend, home, x, y, ctlport, sensorport):
    ctlser = serial.Serial(ctlport, baudrate=250000)
    sensorser = serial.Serial(sensorport, baudrate=115200)
    
    if home:
        cmd(ctlser, "G28")
        
    cmd(ctlser, "G0 Z{}".format(zstart))
    cmd(ctlser, "G0 X{} Y{} F2000".format(x,y))
    

if __name__ == '__main__':
    main()
