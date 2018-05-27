from pipetools import pipe
from plotting.svg2lines import ServodPenGcodeEmitter, overdraw_path_coll, sort_path, Rectangle, convert_ps_to_pltme, \
    convert_svg_to_pltme, parse_pltme, Path, PathCollection
from matplotlib import pyplot as plt
from matplotlib import patches
from plotting.fill_old import raster_polygon


def display(plater, stock, paths):
    plt.figure(figsize=(10, 10))
    # Plater as background
    plt.gca().add_patch(
        patches.Rectangle([plater.xmin, plater.ymin], plater.width, plater.height, color=[0.4, 0.7, 1.0]))
    plt.gca().add_patch(patches.Rectangle([stock.xmin, stock.ymin], stock.width, stock.height, color=[0.8, 0.3, 0.0]))
    for p in paths:
        plt.plot(p.x, p.y, '-k')
    plt.gca().set_aspect('equal')
    plt.show()


def fill_paths(pc: PathCollection, fill_distance, merge_distance) -> PathCollection:
    out = []
    for path in pc.paths:
        if path.style.lower() in ('fill', 'eofill'):
            print(path.color)
            out.extend((Path(x=p.xs, y=p.ys, style=None, color=None) for p in polyfill.raster_merge_polygon(path.x, path.y, fill_distance, merge_distance)))
        else:
            out.append(path)
    return PathCollection(out)


plater = Rectangle(xmin=0, width=200, ymin=0, height=200)
stock = Rectangle(xmin=70, width=100, ymin=20, height=40)
overdraw_amount = 0.5  # g-code units, i.e. mm
fill_distance = 1.0  # mm, distance between filled polygon interior lines
merge_distance = 1.1  # mm, distance between filled polygon interior lines
gcode_emitter = ServodPenGcodeEmitter(safe_z=100.0, working_z=80.0, pin_number=24, value_up=0, value_down=1,
                                      dwell_down_ms=500, dwell_up_ms=500)
#PS_FILEPATH = '/home/moe/dev/plotter/images/out.eps'
# input_pc = parse_pltme(convert_ps_to_pltme(PS_FILEPATH))
input_pc = parse_pltme(convert_svg_to_pltme('tri.svg'))
transformed_pc = input_pc.scale(0.3).translate(0, 0)
filled = fill_paths(transformed_pc, fill_distance, merge_distance)
overdrawn = overdraw_path_coll(filled, 0.5)
sorted_paths = sort_path(overdrawn)
output_pc = sorted_paths

display(plater, stock, output_pc.paths)

gcode_lines = gcode_emitter.generate(output_pc)
with open('holbein.gcode', 'w') as f:
    f.writelines(l + '\n' for l in gcode_lines)
