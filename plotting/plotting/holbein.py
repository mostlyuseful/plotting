from pipetools import pipe
from plotting.svg2lines import ServodPenGcodeEmitter, overdraw_path_coll, Rectangle, convert_ps_to_pltme, \
    convert_svg_to_pltme, parse_pltme, SingleStrokePath, PathCollection, PltmePath, PltmePathGroup
from matplotlib import pyplot as plt
from matplotlib import patches
from plotting.polyfill._polyfill import raster_merge_polygon_eo, simplify_polygon
from plotting.fill_pyclipper import raster_polygon
from plotting.merge import merge_close_paths
from plotting.sort_paths import sort_path_coll
from typing import Iterable
import numpy as np
import json

from matplotlib.collections import LineCollection


def display(plater, stock, paths):
    plt.figure(figsize=(10, 10))
    # Plater as background
    plt.gca().add_patch(
        patches.Rectangle([plater.xmin, plater.ymin], plater.width, plater.height, color=[0.4, 0.7, 1.0]))
    plt.gca().add_patch(patches.Rectangle([stock.xmin, stock.ymin], stock.width, stock.height, color=[0.8, 0.3, 0.0]))
    #for p in paths:
    #    plt.plot(p.x, p.y, '-k')
    def as_segment(p):
      return [(x,y) for (x,y) in zip(p.x, p.y)]
      
    segments = [ as_segment(p) for p in paths ]    
    plt.gca().add_collection(LineCollection(segments, colors=[(0.0,0.0,0.0)]))
    
    plt.gca().set_aspect('equal')
    plt.gca().set_xlim(plater.xmin, plater.xmin+plater.width)
    plt.gca().set_ylim(plater.ymin, plater.ymin+plater.height)
    plt.tight_layout(pad=0.1, h_pad=0, w_pad=0)
    plt.show()


def pltmepath2sspath(input: PltmePath, style, color) -> SingleStrokePath:
    xx, yy = np.asarray(input.coordinates).T
    return SingleStrokePath(xx, yy, style, color)

def num_edges(poly_paths):
    return sum(len(path) for path in poly_paths)

def raster_merge_polygon(poly_paths, dy: float, max_merge_distance: float):
    print("Rastering {} edges".format(num_edges(poly_paths)))
    rastered = raster_polygon(poly_paths, dy)
    print("DONE: raster_polygon")
    merged_paths = merge_close_paths(poly_paths, rastered, max_merge_distance)
    print("DONE: merge_close_paths")
    return merged_paths


def store(paths):
    out_fn = "paths_{:06}.json".format(store.count)
    store.count += 1
    with open(out_fn, 'w') as f:
        json.dump(paths, f, indent=2)
store.count = 0

def convert_paths(path_groups: Iterable[PltmePathGroup], fill_distance, merge_distance) -> PathCollection:
    out = []
    for i,group in enumerate(path_groups):
        if group.style.upper() in ('FILL', 'EOFILL'):
            # rastered = raster_merge_polygon([p.coordinates for p in group.paths], fill_distance, merge_distance)
            # rastered = raster_polygon([p.coordinates for p in group.paths], fill_distance)
            if simplify:
                simplified = simplify_polygon([p.coordinates for p in group.paths], simplify_amount)
                simplified = [path for path in simplified if path]
            else:
                simplified = [p.coordinates for p in group.paths]
            print("#{} rastering {} edges".format(i, num_edges(simplified)))
            # store(simplified)
            rastered = raster_merge_polygon_eo(simplified, fill_distance, merge_distance)
            out.extend(SingleStrokePath(r.xs, r.ys, group.style, group.color) for r in rastered)
            print(len(rastered))
            out.extend(SingleStrokePath.from_zipped(xy, group.style, group.color) for xy in simplified if xy)
        else:
            out.extend(pltmepath2sspath(p, group.style, group.color) for p in group.paths)
    return PathCollection(out)


plater = Rectangle(xmin=0, width=200, ymin=0, height=200)
stock = Rectangle(xmin=70, width=100, ymin=20, height=40)
simplify = False
simplify_amount = 0.01
overdraw_amount = 0.5  # g-code units, i.e. mm
fill_distance = 0.1 # mm, distance between filled polygon interior lines
merge_distance = 10  # mm, distance between filled polygon interior lines
sort_paths = True
gcode_emitter = ServodPenGcodeEmitter(safe_z=100.0, working_z=80.0, pin_number=24, value_up=0, value_down=1,
                                      dwell_down_ms=500, dwell_up_ms=500)
#input_pc = parse_pltme(convert_ps_to_pltme('/home/moe/dev/plotter/images/out.eps'))
input_pc = parse_pltme(convert_svg_to_pltme('tri.svg'))

if 0:
    for group in input_pc:
        print("[")
        for path in group.paths:
            print(path.coordinates, ",")
        print("],")
    1/0

transformed_pc = [group.scale(0.3).translate(100, 25) for group in input_pc]
filled = convert_paths(transformed_pc, fill_distance, merge_distance)
overdrawn = overdraw_path_coll(filled, 0.5)
if sort_paths:
    print('Sorting paths',flush=True)
    sorted_paths = sort_path_coll(overdrawn)
    print('Paths sorted', flush=True)
    output_pc = sorted_paths
else:
    output_pc = overdrawn

display(plater, stock, output_pc.paths)

gcode_lines = gcode_emitter.generate(output_pc)
with open('holbein.gcode', 'w') as f:
    f.writelines(l + '\n' for l in gcode_lines)
