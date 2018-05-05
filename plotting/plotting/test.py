from timeit import timeit
import pyxfill
xx = [1, 2, 3, 0.3, 3.7, 1]
yy = [0, 3, 0, 2, 2, 0]
pgon = pyxfill.Polygon(xx, yy)

n = 50

print(timeit(stmt="""
for y, spans in pyxfill.raster_polygon(pgon, 0.01):
    _ = list(spans)
""", number=n, globals=globals())/n*1e3,"ms")