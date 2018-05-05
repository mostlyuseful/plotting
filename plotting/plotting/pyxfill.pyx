# distutils: language = c++

import numpy as np
cimport numpy as np

from libcpp.vector cimport vector

cdef extern from "polyfill.hpp":
  raster_polygon_iface()