# coding: utf-8

import _polyfill as polyfill
import numpy as np
from matplotlib import pyplot as plt
from timeit import timeit

xx = [1, 2, 3, 0.3, 3.7, 1]
yy = [0, 3, 0, 2, 2, 0]

#xx = np.asarray([0, 1, 0, 0]) + 2
#yy = np.asarray([0, 1, 2, 0]) + 10

#xx = np.asarray([10, 20, 20, 10, 10])
#yy = np.asarray([100, 100, 96, 96, 100])

N = 100
print(timeit("polyfill.raster_polygon(xx,yy, 0.05)",
             number=N, globals=globals())/N*1e6, "µs")

plt.figure()
plt.plot(xx, yy, 'x-r')

mode = 1

if mode==0:
    for line in polyfill.raster_polygon(xx, yy, 0.2):
        y, spans = line.y, line.spans
        for span in spans:
            plt.plot([span.x0, span.x1], [y, y], 'x-b')

if mode==1:
    for path in polyfill.raster_merge_polygon(xx, yy, 0.2, 0.3):
        plt.plot(path.xs, path.ys, 'x-b')

plt.axis('equal')
plt.show()
