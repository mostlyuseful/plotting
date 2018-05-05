from collections import namedtuple
from pipetools import pipe
from typing import Union, List
from enum import Enum
import numpy as np

Point = namedtuple('Point', ['x', 'y'])
ParametricLine = namedtuple('ParametricLine', ['p', 'u'])
Line = namedtuple('Line', ['homogenous', 'parametric', 'p1', 'p2'])


class DenormalizedIntersection(Enum):
    nowhere = 0
    everywhere = 1


IntersectionPoint = namedtuple('IntersectionPoint', ['location', 'on_l1', 'on_l2'])
Intersection = Union[IntersectionPoint, DenormalizedIntersection]


def homogenous_form(p1, p2) -> np.ndarray:
    return np.cross([p1.x, p1.y, 1.0], [p2.x, p2.y, 1.0])


def parametric_form(p1, p2) -> ParametricLine:
    f = np.asfarray
    # Vektor
    u = f([p2.x - p1.x, p2.y - p1.y])
    # Fußpunkt
    p = f([p1.x, p1.y])
    return ParametricLine(p=p, u=u)


def line_from_endpoints(p1, p2) -> Line:
    return Line(homogenous=homogenous_form(p1, p2),
                parametric=parametric_form(p1, p2),
                p1=p1, p2=p2)


def parametric_eval(pf, s):
    return pf.p + (s * pf.u)


def parametric_solve(pf, p: Point) -> (float, float):
    a = np.matrix(pf.u).T
    b = np.matrix([p.x - pf.p[0], p.y - pf.p[1]]).T
    x, residuals, _, _ = np.linalg.lstsq(a, b, rcond=None)
    return x[0, 0], residuals[0, 0]


def intersection(l1: Line, l2: Line) -> Intersection:
    ix = np.cross(l1.homogenous, l2.homogenous)
    if np.allclose(ix, 0):
        return DenormalizedIntersection.everywhere
    if np.allclose(ix[2], 0):
        return DenormalizedIntersection.nowhere
    # Now check whether point lies on both lines
    ixp = Point(ix[0] / ix[2], ix[1] / ix[2])
    s1, r1 = parametric_solve(l1.parametric, ixp)
    s2, r2 = parametric_solve(l2.parametric, ixp)
    return IntersectionPoint(location=ixp, on_l1=0 <= s1 <= 1, on_l2=0 <= s2 <= 1)


class Polygon(object):
    def __init__(self, xx, yy):
        self.xx = np.asanyarray(xx)
        self.yy = np.asanyarray(yy)

    def y_bounds(self):
        return self.yy.min(), self.yy.max()

    def lines(self):
        n = len(self.xx)
        for i in range(n):
            idx1 = i % n
            idx2 = (i + 1) % n
            p1 = Point(self.xx[idx1], self.yy[idx1])
            p2 = Point(self.xx[idx2], self.yy[idx2])
            yield line_from_endpoints(p1, p2)


Span = namedtuple('Span', ['x0', 'x1'])


def get_spans(polygon: Polygon, y: float):
    scanline = np.cross((0, y, 1), (1, y, 1))
    xx = []
    for polyline in polygon.lines():
        ix = np.cross(scanline, polyline.homogenous)
        if ix[2] == 0:
            continue
        ixp = Point(ix[0] / ix[2], ix[1] / ix[2])
        s, _ = parametric_solve(polyline.parametric, ixp)
        if 0 <= s <= 1:
            xx.append(ixp.x)
    xx = sorted(set(xx))
    for a, b in zip(xx[::2], xx[1::2]):
        yield Span(x0=a, x1=b)


class Edge(object):
    def __init__(self, line, ymin, ymax, slope, last_intersection_point):
        self.line = line
        self.ymin = ymin
        self.ymax = ymax
        self.slope = slope
        self.last_intersection_point = last_intersection_point


def to_edge(l: Line) -> Edge:
    ymin = min(l.p1.y, l.p2.y)
    ymax = max(l.p1.y, l.p2.y)
    return Edge(line=l,
                ymin=ymin,
                ymax=ymax,
                slope=inv_slope(l),
                last_intersection_point=None)


def sort_edges(p: Polygon):
    lines = list(p.lines())

    def bottom(line: Line) -> float:
        return min(line.p1.y, line.p2.y)

    return [to_edge(l) for l in sorted(lines, key=bottom)]


class Horizontal(object):
    pass


class Vertical(object):
    pass


Slope = Union[Horizontal, Vertical, float]


def inv_slope(l: Line) -> Slope:
    dy = l.p2.y - l.p1.y
    dx = l.p2.x - l.p1.x
    if dy == 0:
        return Horizontal()
    elif dx == 0:
        return 0
    else:
        return dx / dy


def discard_stale_edges(active_edges: List[Edge], y: float):
    print("y =",y)
    print('\n'.join("{},{}".format(e.ymin, e.ymax) for e in active_edges))
    return [e for e in active_edges if e.ymin < y <= e.ymax]


def fill_active_edges(remaining_edges, active_edges, y):
    new_remaining_edges = remaining_edges[::]
    new_active_edges = active_edges[::]
    while new_remaining_edges:
        front = new_remaining_edges[0]  # type: Edge
        if front.ymin < y <= front.ymax:
            new_active_edges.append(new_remaining_edges.pop(0))
        else:
            break
    return new_remaining_edges, new_active_edges


def raster_polygon(p: Polygon, dy: float = 1.0):
    remaining_edges = sort_edges(p)
    active_edges = []

    ymin, ymax = p.y_bounds()
    for y in np.arange(ymin + dy, ymax + dy, dy):
        active_edges = discard_stale_edges(active_edges, y)
        remaining_edges, active_edges = fill_active_edges(remaining_edges, active_edges, y)
        intersections = []
        for e in active_edges:
            if e.last_intersection_point is None:
                scanline = np.cross((0, y, 1), (1, y, 1))
                ix = np.cross(scanline, e.line.homogenous)
                if ix[2] != 0:
                    ixp = Point(ix[0] / ix[2], ix[1] / ix[2])
                    s, _ = parametric_solve(e.line.parametric, ixp)
                    if 0 <= s <= 1:
                        intersections.append(ixp.x)
                        e.last_intersection_point = ixp
            else:
                old_ixp = e.last_intersection_point
                local_dy = y - old_ixp.y
                new_ixp = Point(x=local_dy * e.slope + old_ixp.x, y=y)
                intersections.append(new_ixp.x)
                e.last_intersection_point = new_ixp
        xx = sorted(set(intersections))
        spans = []
        for a, b in zip(xx[::2], xx[1::2]):
            spans.append(Span(x0=a, x1=b))
        yield y, spans


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    plt.figure()

    xx = [1, 2, 3, 0.3, 3.7, 1]
    yy = [0, 3, 0, 2, 2, 0]
    pgon = Polygon(xx, yy)

    plt.plot(xx, yy, 'x-r')

    for y, spans in raster_polygon(pgon, 0.125):
        for span in spans:
            plt.plot([span.x0, span.x1], [y, y], 'x-b')

    plt.axis('equal')
    plt.show()
