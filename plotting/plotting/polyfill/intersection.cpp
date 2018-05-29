#include "intersection.hpp"

Intersection intersect_lines(Line const& l1, Line const& l2) {
    Eigen::Array3d const ix = l1.homogenous.matrix().cross(l2.homogenous.matrix()).array();
    if(ix.isApproxToConstant(0)) {
        return DenormalIntersection::everywhere;
    } else if(ix.tail<1>().isApproxToConstant(0)) {
        return DenormalIntersection::nowhere;
    }
    // Now check whether point lies on both lines
    Eigen::Array2d const location(ix(0) / ix(2), ix(1) / ix(2));
    auto const s1 = l1.parametric.solve_for_s(location);
    auto const s2 = l2.parametric.solve_for_s(location);
    bool const on_l1 = (s1>=0) && (s1<=1);
    bool const on_l2 = (s2>=0) && (s2<=1);
    return IntersectionPoint{location, on_l1, on_l2};
}
