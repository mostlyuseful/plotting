import numpy as np
import pyclipper
import itertools
from collections import namedtuple

Bounds = namedtuple('Bounds', ['min_x', 'max_x', 'min_y', 'max_y'])


def bounds_path(path):
    xx, yy = np.asarray(path).T
    return Bounds(np.min(xx), np.max(xx), np.min(yy), np.max(yy))


def bounds_paths(paths):
    sub_bounds = [bounds_path(p) for p in paths]
    min_x = min(_.min_x for _ in sub_bounds)
    max_x = max(_.max_x for _ in sub_bounds)
    min_y = min(_.min_y for _ in sub_bounds)
    max_y = max(_.max_y for _ in sub_bounds)
    return Bounds(min_x, max_x, min_y, max_y)


def generate_lines_pattern(bounds: Bounds, dy: float):
    paths = [((bounds.min_x, y), (bounds.max_x, y)) for y in np.arange(bounds.min_y, bounds.max_y, dy)]
    return paths


def scale_paths(paths, s):
    out_paths = []
    for p in paths:
        out_path = []
        for (x, y) in p:
            out_path.append((x * s, y * s))
        out_paths.append(out_path)
    return out_paths


def raster_polygon(poly_paths, dy: float = 1.0):
    path_scale = 1000
    blown_up_dy = dy * path_scale
    blown_up_poly_paths = scale_paths(poly_paths, path_scale)
    blown_up_lines = generate_lines_pattern(bounds_paths(blown_up_poly_paths), blown_up_dy)
    pc = pyclipper.Pyclipper()
    pc.AddPaths(blown_up_poly_paths, pyclipper.PT_CLIP, True)
    pc.AddPaths(blown_up_lines, pyclipper.PT_SUBJECT, False)
    solution = pc.Execute2(pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
    assert solution.depth <= 1
    out_paths = []
    for child in solution.Childs:
        assert child.IsOpen
        xx, yy = np.asarray(child.Contour).T / path_scale
        out_path = [(x, y) for x, y in zip(xx, yy)]
        out_paths.append(out_path)
    return out_paths
