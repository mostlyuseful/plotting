# coding: utf-8

from matplotlib import pyplot as plt
from plotting import pyxfill

xx = [1, 2, 3, 0.3, 3.7, 1]
yy = [0, 3, 0, 2, 2, 0]

plt.figure()
plt.plot(xx, yy, 'x-r')

for y, spans in pyxfill.raster_polygon(pgon, 0.125):
    for span in spans:
        plt.plot([span.x0, span.x1], [y, y], 'x-b')

plt.axis('equal')
plt.show()
