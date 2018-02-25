#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from mypy.types import Union

def move_check_hit(z1, z2, travel_speed=9999):
    go(z=z1, speed=travel_speed).wait()
    go(z=z2, speed=travel_speed)
    while is_moving():
        pass


EndstopNotFound = object()
EndstopResult = Union[EndstopNotFound, float]

def find_endstop(z1:float, z2:float, epsilon:float=0.01, was_hit:bool=False) -> EndstopResult :
    zmin, zmax = sorted([z1,z2])
    # Delta-z: Half subdivision span
    dz = (zmax-zmin)/2
    if(dz<epsilon and was_hit):
        return (z2+z1)/2
    if move_check_hit(zmax, zmin+dz):
        return find_endstop(zmax, zmin+dz, epsilon)
    elif move_check_hit(zmin+dz, zmin):
        return find_endstop(zmin+dz, zmin, epsilon)
    else:
        return EndstopNotFound
