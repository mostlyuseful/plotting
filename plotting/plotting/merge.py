import numpy as np
from plotting import svg2lines as s2l


def make_candidate(p, q, q_orig):
    dst = np.hypot(*np.subtract(p.end(), q.start()))
    return dst, p, q, q_orig


def merge_paths(coll, distance_threshold, clipping_paths):
    pool = coll.paths[::]
    while True:
        merged_at_least_one = False
        for p in pool:
            # Also flip p too, check if that helps:
            # Four possible combinations: p+q, p + r(q), r(p) + q, r(p) + r(q)

            min_dst = float('+inf')
            min_p = None
            min_q = None
            min_q_orig = None

            for q in [_ for _ in pool if _ is not p]:

                r = s2l.reversed_path

                candidates = [
                    make_candidate(p, q, q),
                    make_candidate(p, r(q), q),
                    make_candidate(r(p), q, q),
                    make_candidate(r(p), r(q), q)
                ]

                dst, cp, cq, cq_orig = min(candidates, key=lambda c: c[0])
                if dst < min_dst:
                    min_dst = dst
                    min_p = cp
                    min_q = cq
                    min_q_orig = cq_orig
            if min_dst < distance_threshold and min_q is not None:
                pool.remove(p)
                pool.remove(min_q_orig)
                pool.append(min_p.append(min_q))
                merged_at_least_one = True
                break
        if not merged_at_least_one:
            break
