import io
import subprocess
import numpy as np
from collections import namedtuple
from functools import reduce
from typing import Iterable

PSTOEDIT_CMD = '/home/moe/dev/plotter/pstoedit/prefix/bin/pstoedit'


def convert_ps_to_pltme(ps_path, pstoedit_cmd=PSTOEDIT_CMD):
    subprocess.check_call([pstoedit_cmd, '-f', 'plotme',
                           ps_path, '__tmp__.pltme'])
    with io.open('__tmp__.pltme') as f:
        raw = f.read()
    return raw


def convert_svg_to_pltme(svg_path, pstoedit_cmd=PSTOEDIT_CMD):
    subprocess.check_call(['cairosvg', svg_path, '-o', '__tmp__.ps'])
    subprocess.check_call([pstoedit_cmd, '-f', 'plotme',
                           '__tmp__.ps', '__tmp__.pltme'])
    with io.open('__tmp__.pltme') as f:
        raw = f.read()
    return raw


class Path(object):
    __slots__ = ['x', 'y', 'style', 'color']

    def __init__(self, x, y, style, color):
        self.x = x
        self.y = y
        self.style = style
        self.color = color

    def translate(self, dx, dy):
        return Path(x=np.asanyarray(self.x) + dx,
                    y=np.asanyarray(self.y) + dy,
                    style=self.style,
                    color=self.color)

    def scale(self, factor):
        return Path(x=factor * np.asanyarray(self.x),
                    y=factor * np.asanyarray(self.y),
                    style=self.style,
                    color=self.color)

    def length(self):
        dx = np.diff(self.x)
        dy = np.diff(self.y)
        hh = np.hypot(dx, dy)
        return np.sum(hh)

    def start(self):
        return self.x[0], self.y[0]

    def end(self):
        return self.x[-1], self.y[-1]

    def minmax_x(self):
        return min(self.x), max(self.x)

    def minmax_y(self):
        return min(self.y), max(self.y)

    def width(self):
        min_x, max_x = self.minmax_x()
        return max_x - min_x

    def height(self):
        min_y, max_y = self.minmax_y()
        return max_y - min_y

    def append(self, other_path):
        x = np.hstack([self.x, other_path.x])
        y = np.hstack([self.y, other_path.y])
        return Path(x, y, self.style, self.color)

    def __str__(self):
        return '<Path N={} L={}>'.format(len(self.x), self.length())

    __repr__ = __str__


class Rectangle(object):
    __slots__ = ['xmin', 'ymin', 'width', 'height']

    def __init__(self, xmin, ymin, width, height):
        self.xmin = xmin
        self.ymin = ymin
        self.width = width
        self.height = height

    def __contains__(self, other) -> bool:
        return other.xmin >= self.xmin and \
               other.ymin >= self.ymin and \
               other.width <= self.width and \
               other.height <= self.height

    def __str__(self):
        return "<Rectangle {.1f} x {.1f} @ ({.1f}, {.1f})".format(self.width, self.height, self.xmin, self.ymin)

    __repr__ = __str__


class PathCollection(object):
    def __init__(self, paths):
        self.paths = paths

    def minmax_x(self):
        min_x = min(min(p.x) for p in self.paths)
        max_x = max(max(p.x) for p in self.paths)
        return min_x, max_x

    def minmax_y(self):
        min_y = min(min(p.y) for p in self.paths)
        max_y = max(max(p.y) for p in self.paths)
        return min_y, max_y

    def width(self):
        min_x, max_x = self.minmax_x()
        return max_x - min_x

    def height(self):
        min_y, max_y = self.minmax_y()
        return max_y - min_y

    def bounding_box(self) -> Rectangle:
        min_x, _ = self.minmax_x()
        min_y, _ = self.minmax_y()
        return Rectangle(min_x, min_y, self.width(), self.height())

    def scale(self, factor: float):
        return PathCollection([p.scale(factor) for p in self.paths])

    def translate(self, dx, dy):
        return PathCollection([p.translate(dx, dy) for p in self.paths])

    def translate_to_origin(self):
        min_x = min(min(p.x) for p in self.paths)
        min_y = min(min(p.y) for p in self.paths)
        translated_paths = [p.translate(-min_x, -min_y) for p in self.paths]
        return PathCollection(translated_paths)


def parse_color(parts):
    r, g, b = [float(_) for _ in parts]
    return r, g, b


def parse_pltme(source: str) -> PathCollection:
    paths = []
    xx = []
    yy = []
    path_style = None
    color = None

    def output():
        nonlocal xx, yy, paths, path_style, color
        if not xx:
            return
        paths.append(Path(xx[::], yy[::], style=path_style, color=color))
        xx = []
        yy = []

    for line in source.splitlines():
        parts = line.strip().split(';')
        cmd = parts[0].upper()
        if cmd == 'STARTPATH':
            path_style = parts[1].upper()
            color_parts = parts[2:]
            color = parse_color(color_parts[:3])
            output()
        elif cmd == 'MOVETO':
            output()
            xx = [float(parts[1])]
            yy = [float(parts[2])]
        elif cmd == 'LINETO':
            xx.append(float(parts[1]))
            yy.append(float(parts[2]))
        else:
            raise RuntimeError('Unexpected cmd `{}`'.format(cmd))
    output()

    return PathCollection(paths=paths)


def parse_pltme_file(file_path):
    with io.open(file_path) as f:
        raw = f.read()
    return parse_pltme(raw)


GCODE_DEFAULT_PREAMBLE = """
; Set units to mm
G21
; Absolute positioning
G90
; Home
G28
"""


class GcodeEmitter(object):

    def emit_preamble(self):
        raise NotImplementedError("Implement in subclass")

    def emit_engage(self):
        raise NotImplementedError("Implement in subclass")

    def emit_penup(self):
        raise NotImplementedError("Implement in subclass")

    def emit_pendown(self):
        raise NotImplementedError("Implement in subclass")

    def emit_paths(self, paths: Iterable[Path]):
        for p in paths:
            # Move pen to standby height
            yield from self.emit_penup()
            it = zip(p.x, p.y)
            # Move to first point with pen up
            x, y = next(it)
            yield 'G0 X{} Y{} F50000'.format(x, y)
            # Move pen down
            yield from self.emit_pendown()
            # Draw connected lines
            for x, y in it:
                yield 'G0 X{} Y{} F50000'.format(x, y)
            # Move pen to standby height
            yield from self.emit_penup()

    def emit_disengage(self):
        raise NotImplementedError("Implement in subclass")

    def generate(self, path_coll: PathCollection):
        yield from self.emit_preamble()
        yield from self.emit_engage()
        yield from self.emit_paths(path_coll.paths)
        yield from self.emit_disengage()


class FixedPenGcodeEmitter(GcodeEmitter):
    def __init__(self, plater: Rectangle, engage_z: float, pen_up_z: float, pen_down_z: float):
        self.plater = plater
        self.engage_z = engage_z
        self.pen_up_z = pen_up_z
        self.pen_down_z = pen_down_z

    def emit_preamble(self):
        yield from GCODE_DEFAULT_PREAMBLE.splitlines()

    def emit_engage(self):
        # Go to safe height
        yield 'G0 Z{}'.format(self.engage_z)
        # Move to plater center
        plater_mid_x = self.plater.xmin + self.plater.width / 2
        plater_mid_y = self.plater.ymin + self.plater.height / 2
        yield 'G0 X{} Y{} F50000'.format(plater_mid_x, plater_mid_y)
        # Move pen to standby height
        yield 'G0 Z{}'.format(self.pen_up_z)

    def emit_disengage(self):
        yield 'G0 Z{}'.format(self.engage_z)
        yield 'G0 X190 Y199 F9999'

    def emit_penup(self):
        # Move pen to standby height
        yield 'G0 Z{}'.format(self.pen_up_z)

    def emit_pendown(self):
        # Move pen down
        yield 'G0 Z{}'.format(self.pen_down_z)


class ServodPenGcodeEmitter(GcodeEmitter):
    def __init__(self, safe_z: float, working_z: float, pin_number: int, value_up: int, value_down: int,
                 dwell_up_ms: int, dwell_down_ms: int):
        self.safe_z = safe_z
        self.working_z = working_z
        self.pin_number = pin_number
        self.value_up = value_up
        self.value_down = value_down
        self.dwell_up_ms = dwell_up_ms
        self.dwell_down_ms = dwell_down_ms

    def emit_preamble(self):
        yield from GCODE_DEFAULT_PREAMBLE.splitlines()

    def emit_engage(self):
        # Go to safe height
        yield 'G0 Z{}'.format(self.safe_z)
        yield 'G0 X100 Y100 F9999'
        yield 'G0 Z{}'.format(self.working_z)

    def emit_disengage(self):
        yield 'G0 Z{}'.format(self.safe_z)
        yield 'G0 X190 Y199 F9999'

    def emit_penup(self):
        # Servo pen up
        yield 'M42 P{} S{}'.format(self.pin_number, self.value_up)
        # Wait until pen has broken contact
        yield 'G4 P{}'.format(self.dwell_up_ms)

    def emit_pendown(self):
        # Go to working height
        yield 'G0 Z{}'.format(self.working_z)
        # Wait for arrival
        yield 'M400'
        # Servo pen down
        yield 'M42 P{} S{}'.format(self.pin_number, self.value_down)
        # Wait until pen has made contact
        yield 'G4 P{}'.format(self.dwell_down_ms)


Accu = namedtuple("Accu", "distance path")


def distance(p1, p2):
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def find_nearest(last_path, paths):
    end_point = last_path.end()

    def f(accu, path):
        dst = distance(path.start(), end_point)
        if dst < accu.distance:
            return Accu(dst, path)
        else:
            return accu

    nearest = reduce(f, paths, Accu(float("inf"), None)).path
    return nearest


def sort_path(path_coll):
    available = set(path_coll.paths)
    out = [available.pop()]
    # first = path_coll.paths[20]
    # out = [first]
    # available.remove(first)
    while available:
        last_path = out[-1]
        next_path = find_nearest(last_path, available)
        available.remove(next_path)
        out.append(next_path)
    return PathCollection(out)


def is_path_closed(path):
    if len(path.x) <= 2:
        return False
    x0, x1 = path.x[0], path.x[-1]
    y0, y1 = path.y[0], path.y[-1]
    p0 = x0, y0
    p1 = x1, y1
    d = distance(p0, p1)
    return d < 1e-3


def overdraw_path(path, amount):
    if (not is_path_closed(path)):
        return path
    out_x = np.copy(path.x).tolist()  # type: list
    out_y = np.copy(path.y).tolist()  # type: list
    remaining = amount
    while remaining > path.length():
        remaining -= path.length()
        out_x.extend(path.x)
        out_y.extend(path.y)
    for i, (p1, p2) in enumerate(zip(zip(path.x[:-1], path.y[:-1]), zip(path.x[1:], path.y[1:]))):
        # print(p1,p2)
        d = distance(p1, p2)
        # print("d =",d)
        if remaining > d:
            remaining -= d
            out_x.append(p2[0])
            out_y.append(p2[1])
        else:
            break
    # print("remaining =",remaining)
    if remaining > 0:
        # print("i =",i)
        n = len(path.x)
        last_x = out_x[-1]
        last_y = out_y[-1]
        next_x = path.x[(i + 1) % n]
        next_y = path.y[(i + 1) % n]
        d = float(np.hypot([next_x - last_x], [next_y - last_y]))
        x = np.interp(remaining, [0, d], [last_x, next_x])
        y = np.interp(remaining, [0, d], [last_y, next_y])
        out_x.append(x)
        out_y.append(y)
    return Path(out_x, out_y, path.style, path.color)


def overdraw_path_coll(path_coll, amount):
    opaths = [overdraw_path(path, amount) for path in path_coll.paths]
    return PathCollection(opaths)


def filling_pattern(width, height, delta):
    def gen():
        while True:
            yield [0, width]
            yield [width, 0]

    def gen2():
        y = 0
        while True:
            yield [y, y]
            # yield [y, y+delta]
            y += delta

    path = Path([], [], None, None)
    y = 0.0
    g = gen()
    g2 = gen2()
    while y < height:
        path.x.extend(next(g))
        path.y.extend(next(g2))
        y += delta
    return path


def reversed_path(path: Path):
    return Path(path.x[::-1], path.y[::-1], path.style, path.color)
