# coding: utf-8

from matplotlib import pyplot as plt
import polyfill
from timeit import timeit

xx = [1, 2, 3, 0.3, 3.7, 1]
yy = [0, 3, 0, 2, 2, 0]

#xx = [0,1,0,0]
#yy = [0,1,2,0]

N = 100
print(timeit("polyfill.raster_polygon(xx,yy, 0.05)", number=N, globals=globals())/N*1e6,"µs")

plt.figure()
plt.plot(xx, yy, 'x-r')

for line in polyfill.raster_polygon(xx,yy, 0.05):
    y, spans = line.y, line.spans
    for span in spans:
        plt.plot([span.x0, span.x1], [y, y], 'x-b')

plt.axis('equal')
plt.show()
