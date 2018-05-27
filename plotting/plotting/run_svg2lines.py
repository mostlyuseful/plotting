from pipetools import pipe
from plotting.svg2lines import ServodPenGcodeEmitter, overdraw_path_coll, sort_path, Rectangle, convert_svg_to_pltme, \
    parse_pltme, SingleStrokePath, PathCollection
from matplotlib import pyplot as plt
from matplotlib import patches

def display(paths):
    plt.figure(figsize=(10, 10))
    # Plater as background
    plt.gca().add_patch(
        patches.Rectangle([plater.xmin, plater.ymin], plater.width, plater.height, color=[0.4, 0.7, 1.0]))
    for p in paths:
        plt.plot(p.x, p.y, '-k')
    plt.gca().set_aspect('equal')
    plt.show()


path_coll = parse_pltme(convert_svg_to_pltme('pocmon.svg')).scale(0.3)
gcode_emitter = ServodPenGcodeEmitter(safe_z=100.0, working_z=80.0, pin_number=24, value_up=0, value_down=1,
                                      dwell_down_ms=500, dwell_up_ms=500)
plater = Rectangle(xmin=10, width=190, ymin=60, height=140)
overdraw_amount = 0.5  # g-code units, i.e. mm

prepare = pipe \
          | (lambda pc: overdraw_path_coll(pc, 0.5)) \
          | sort_path \
          | (lambda pc: pc.translate_to_origin()) \
          | (lambda pc: pc.translate(plater.xmin, plater.ymin))

prepared_pc = prepare(path_coll)

display(prepared_pc.paths)

if prepared_pc.bounding_box() in plater:
    print("Drawing completely in bounds")
else:
    print("Drawing cannot be realized as some parts are out of bounds")
    1 / 0

gcode_lines = gcode_emitter.generate(prepared_pc)
with open('pocmon.gcode', 'w') as f:
    f.writelines(l + '\n' for l in gcode_lines)
