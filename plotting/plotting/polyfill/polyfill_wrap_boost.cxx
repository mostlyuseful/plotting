#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

#include "polygon.hpp"

#include <vector>

using VDouble = std::vector<double>;

BOOST_PYTHON_MODULE(_polyfill)
{
    /*class_<Polygon>("Polygon")
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;*/

    class_<VDouble>("VDouble").def(vector_indexing_suite<VDouble>());

    def("raster_polygon", raster_polygon);
}
