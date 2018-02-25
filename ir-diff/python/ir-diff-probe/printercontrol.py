#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import numpy as np
import rx.subjects
from serial import Serial

printer_port = '/dev/ttyUSB0'
printer_serial = Serial(printer_port, baudrate=250000)
printer_sub = rx.subjects.Subject()

sensor_port = '/dev/ttyACM0'
sensor_serial = Serial(sensor_port, baudrate=115200)
sensor_sub = rx.subjects.Subject()

def read_lines_printer():
    while True:
        printer_sub.on_next(printer_serial.readline())


def read_lines_sensor():
    while True:
        sensor_sub.on_next(sensor_serial.readline())


tprint = threading.Thread(target=read_lines_printer)
tprint.daemon = True
tprint.start()

tsensor = threading.Thread(target=read_lines_sensor)
tsensor.daemon = True
tsensor.start()


def parse(line: bytes):
    parts = line.strip().split(b';')
    ambient, u, v = [int(_) for _ in parts]
    return ambient, u, v


def minus_ambient(tup):
    amb, u, v = tup
    return u - amb, v - amb


def armed(tup):
    Tmin = 100
    u, v = tup
    return (u > Tmin) or (v > Tmin)


def diff(tup):
    u, v = tup
    return u - v


diff_obs = sensor_sub \
    .map(parse) \
    .map(minus_ambient) \
    .filter(armed) \
    .map(diff)


def sign_changed_or_zero(diffs):
    d0, d1 = diffs
    if abs(d0) <= 1:
        return True
    elif np.sign(d0) != np.sign(d1):
        return True
    else:
        return False


trigger_obs = diff_obs \
    .buffer_with_count(count=2, skip=1) \
    .filter(sign_changed_or_zero) \
    .timestamp()


# trigger_obs.subscribe(lambda ts: print(ts.timestamp, ts.value))
# sleep(100)

def wait_for_ok(count=1):
    def is_ok(s: bytes):
        return s.strip().lower() == b'ok'

    it = printer_sub \
        .filter(is_ok) \
        .take(count) \
        .to_blocking() \
        .to_iterable()
    return list(it)


def cmd(command, do_print=False):
    if type(command) != bytes:
        command = command.encode()
    if do_print:
        print('CMD: {}'.format(command), flush=True)
    printer_serial.write(command)
    printer_serial.write(b'\r\n')
    printer_serial.flush()
    # if do_print:
    #    s = printer_sub.observe_on(rx.concurrency.EventLoopScheduler()).subscribe(print)
    wait_for_ok()
    # if do_print:
    #    s.dispose()


def go(x=None, y=None, z=None, speed=None):
    tmpl = 'G0'
    if x is not None:
        tmpl += ' X{x}'
    if y is not None:
        tmpl += ' Y{y}'
    if z is not None:
        tmpl += ' Z{z}'
    if speed is not None:
        tmpl += ' F{speed}'
    s = tmpl.format(x=x, y=y, z=z, speed=speed)
    # printer_serial.write(s.encode())
    # printer_serial.write('G4 P0')
    # print(wait_for_ok(2))
    cmd(s)
    cmd('G4 P0')

def antasten(z_ceiling, z_floor, dz_step):
    go(z=z_ceiling, speed=9999)

    n = int((np.abs(z_ceiling - z_floor) / dz_step) + 0.5) + 1
    zz = np.linspace(z_ceiling, z_floor, num=n)
    was_hit = False

    def set_hit(*args, **kwargs):
        nonlocal was_hit
        was_hit = True

    result = None

    s = trigger_obs.subscribe(set_hit)
    for z_from, z_to in zip(zz[:-1], zz[1:]):
        # print('z_from={} z_to={}'.format(z_from, z_to))
        go(z=z_to)
        if was_hit:
            # print("Hit between {} and {}".format(z_from, z_to))
            result = z_from, z_to
            break
        else:
            # print("No hit between {} and {}".format(z_from, z_to))
            pass
    s.dispose()
    return result


def hit_between(z0, z1):
    was_hit = False

    def set_hit(*args, **kwargs):
        nonlocal was_hit
        was_hit = True

    go(z=z0)
    s = trigger_obs.subscribe(set_hit)
    go(z=z1)
    s.dispose()
    # if was_hit:
    #    print('Hit between {} and {}'.format(z0, z1))
    return was_hit


def subdivide(z0, z1, dz_min=0.001, _was_hit=False):
    z0, z1 = sorted([z0, z1])
    # print('Subdivide from {} to {}'.format(z0, z1))
    span = z1 - z0
    dz_half = span / 2
    if span < dz_min:
        # print('span < dz_min, was_hit={}'.format(_was_hit))
        return z0, z1 if _was_hit else None

    if hit_between(z1, z1 - dz_half):
        return subdivide(z1, z1 - dz_half, dz_min, True)
    elif hit_between(z0 + dz_half, z0):
        return subdivide(z0 + dz_half, z0, dz_min, True)
    else:
        print('Whoopsie case')
        if _was_hit:
            return z0, z1
        else:
            return None


# Set units to mm
cmd('G21')
# Absolute positioning
cmd('G90')


def xyz():
    go(x=100, y=100, speed=9999)
    z0, z1 = antasten(7.0, 0.1, 1.0)
    print('Nach antasten: z0={}, z1={}'.format(z0, z1))
    z_final = subdivide(z0, z1)
    print('Finales Ergebnis: z={}'.format(z_final))


#go(x=50, y=199, speed=9999)
#print(subdivide(5,1))
#1/0

#cmd('G28')
z_ceiling = 5.0
z_floor = 0.1
go(z=z_ceiling)

# for x in np.linspace(50, 200, 3):
for x in [50, 190]:
    for y in [60, 199]:
        # for y in np.linspace(50, 200, 3):
        go(x=x, y=y, speed=9999)
        z0, z1 = antasten(z_ceiling, z_floor, 2)
        z_final = subdivide(z0, z1)
        print('x={} ; y={} ; z_final={}'.format(x, y, np.mean(z_final)))
