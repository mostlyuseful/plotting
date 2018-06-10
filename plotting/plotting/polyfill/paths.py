from pylab import *
from plotting.polyfill import _polyfill as _p
import json

with open("/home/moe/dev/plotter/plotting/plotting/paths_000000.json") as f:
    paths = json.load(f)

if 0:
    figure()
    axis('equal')
    for path in paths:
        xx,yy = np.asarray(path).T
        plot(xx,yy,'-')


fill_distance = 19.0
merge_distance = 2*fill_distance
rastered = _p.raster_merge_polygon_eo(paths, fill_distance, merge_distance)

figure(figsize=(10,10))
axis('equal')
for path in paths:
    xx,yy = np.asarray(path).T
    plot(xx,yy,'-k')
for r in rastered:
    plot(r.xs, r.ys, 'x-r')
xlim(10,20)

