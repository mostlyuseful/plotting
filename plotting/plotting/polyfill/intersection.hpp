#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

#include "line.hpp"
#include <boost/variant.hpp>
#include <eigen3/Eigen/Dense>

struct IntersectionPoint {
    Eigen::Array2d location;
    bool on_l1;
    bool on_l2;
    double s1;
    double s2;
};

enum class DenormalIntersection {
    nowhere,
    everywhere
};

using Intersection = boost::variant<IntersectionPoint, DenormalIntersection>;

Intersection intersect_lines(Line const& l1, Line const& l2);

#endif
