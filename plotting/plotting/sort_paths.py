import numpy as np
from collections import namedtuple
from plotting.svg2lines import PathCollection
from rtree import index
from time import time

from plotting.polyfill._polyfill import sort_paths


def first_or_none(seq):
    for it in seq:
        return it
    return None


def sort_path_coll_old(path_coll: PathCollection):
    print("Sorting {} paths".format(len(path_coll.paths)))

    def gen():
        for i, p in enumerate(path_coll.paths[1:]):
            x, y = p.start()
            yield i, (x, y, x, y), p
    rtree = index.Index(gen())

    out = [path_coll.paths[0]]

    #counter = 0
    while True:
        #counter += 1
        #if counter%100==0: print(counter)
        last_path = out[-1]
        end_point = last_path.end()
        hits = rtree.nearest(
            (end_point[0], end_point[1], end_point[0], end_point[1]), 1, True)
        hit = first_or_none(hits)
        if hit is not None:
            next_path = hit.object
            rtree.delete(hit.id, hit.bbox)
            out.append(next_path)
        else:
            break

    return PathCollection(out)

def sort_path_coll(path_coll: PathCollection):
    print("Sorting {} paths".format(len(path_coll.paths)))
    t0=time()
    endpoints = [(p.start(),p.end()) for p in path_coll.paths]
    indices = sort_paths(endpoints)    
    shuffled_paths = [path_coll.paths[i] for i in indices]
    dt = time()-t0
    print(dt*1e3,'ms')
    return PathCollection(shuffled_paths)
    

