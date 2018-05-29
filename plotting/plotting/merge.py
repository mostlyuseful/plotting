import numpy as np
import pyclipper
from plotting import svg2lines as s2l
from .fill_pyclipper import scale_paths


def make_candidate(p, q, q_orig):
    p_end = p[-1]
    q_start = q[0]
    dst = np.hypot(*np.subtract(p_end, q_start))
    return dst, p, q, q_orig


class Candidate(object):
    def __init__(self, tup):
        self.tup = tup

    @property
    def dst(self):
        return self.tup[0]

    @property
    def p(self):
        return self.tup[1]

    @property
    def q(self):
        return self.tup[2]

    @property
    def q_orig(self):
        return self.tup[3]


def merge_close_paths(polygon_paths, paths, max_merge_distance):
    # ClipperLib uses integer math, we use floating-point.
    # Multiply our coordinates before using them in ClipperLib, a naive version of fixed-point math
    blow_up_factor = 1000
    blown_up_polygon_paths = scale_paths(polygon_paths, blow_up_factor)

    def violates_polygon(c: Candidate) -> bool:
        P = c.p[-1]
        Q = c.q[0]
        blown_up_connector_line = [
            (P[0] * blow_up_factor, P[1] * blow_up_factor),
            (Q[0] * blow_up_factor, Q[1] * blow_up_factor)
        ]
        clipper = pyclipper.Pyclipper()
        clipper.AddPaths(blown_up_polygon_paths, pyclipper.PT_CLIP, True)
        clipper.AddPath(blown_up_connector_line, pyclipper.PT_SUBJECT, False)
        solution = clipper.Execute2(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        # If something remains after clipping, they are lying outside the polygon.
        return solution.depth != 0 or len(solution.Contour) != 0

    pool = paths[::]
    while True:
        merged_at_least_one = False
        for p in pool:
            # Also flip p too, check if that helps:
            # Four possible combinations: p+q, p + r(q), r(p) + q, r(p) + r(q)

            min_dst = float('+inf')
            min_p = None
            min_q = None
            min_q_orig = None

            for q in enumerate([_ for _ in pool if _ is not p]):
                def r(l):
                    return l[::-1]

                all_candidates = [
                    Candidate(c) for c in [
                        make_candidate(p, q, q),
                        make_candidate(p, r(q), q),
                        make_candidate(r(p), q, q),
                        make_candidate(r(p), r(q), q)
                    ]
                ]

                inside_candidates = [c for c in all_candidates if not violates_polygon(c)]
                if inside_candidates:
                    it = min(inside_candidates, key=lambda a: a.dst)
                    if it.dst < min_dst:
                        min_dst = it.dst
                        min_p = it.p
                        min_q = it.q
                        min_q_orig = it.q_orig
            if min_dst < max_merge_distance and min_q is not None:
                pool.remove(p)
                pool.remove(min_q_orig)
                pool.append(min_p.append(min_q))
                merged_at_least_one = True
                break
        if not merged_at_least_one:
            break
    return pool
