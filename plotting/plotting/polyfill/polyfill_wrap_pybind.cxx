#include "simpleapi.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <vector>

namespace py = pybind11;

PYBIND11_MODULE(_polyfill, m)
{
    m.def("raster_polygon", &raster_polygon);
    m.def("raster_merge_polygon", &raster_merge_polygon);
    
    py::class_<RasterLine>(m, "RasterLine")
        .def_readonly("spans", &RasterLine::spans)
        .def_readonly("y", &RasterLine::y);
    
    py::class_<Span>(m, "Span")
        .def_readonly("x0", &Span::x0)
        .def_readonly("x1", &Span::x1);

    py::class_<Path>(m, "Path")
        .def_readonly("xs", &Path::xs)
        .def_readonly("ys", &Path::ys);
}
