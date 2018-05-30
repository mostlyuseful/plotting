cdef extern from "pnpoly.h":
    int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
    
# ::1 means C-contiguous
def pt_in_poly(float [::1] vertx, float [::1] verty, const float testx, const float testy):
    cdef Py_ssize_t nvert = vertx.shape[0]
    cdef int in_poly = pnpoly(nvert, &vertx[0], &verty[0], testx, testy)
    return in_poly