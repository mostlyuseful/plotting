#ifndef PNPOLY_RTREE_HPP
#define PNPOLY_RTREE_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace pip {
namespace rtree {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<float, 2, bg::cs::cartesian>;
using Box = bg::model::box<Point>;
using Value = std::pair<Box, Segment>;

class InclusionTester
{
public:
    inline InclusionTester() {

        //bgi::linear<16>, bgi::rstar<16>
        bgi::rtree< value, bgi::quadratic<16> > rtree;

    }
    inline bool contains(Point const& point) {}
    inline bool contains(Segment const& segment) {}
};

}
} // namespace pnpoly

#endif // PNPOLY_RTREE_HPP
