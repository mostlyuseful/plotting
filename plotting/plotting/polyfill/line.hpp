#ifndef LINE_HPP
#define LINE_HPP

#include "parametricline.hpp"
#include "slope.hpp"

#include <eigen3/Eigen/Dense>

Eigen::Array3d homogenous_form(Eigen::Array2d const &p1, Eigen::Array2d const &p2);

struct Line {
    Eigen::Array3d homogenous;
    ParametricLine parametric;
    Eigen::Array2d p1;
    Eigen::Array2d p2;

    static Line from_endpoints(Eigen::Array2d const &p1, Eigen::Array2d const &p2);
    static Line from_endpoints(double const x1, double const y1, double const x2, double const y2);

    Slope inv_slope() const;
};

#endif
