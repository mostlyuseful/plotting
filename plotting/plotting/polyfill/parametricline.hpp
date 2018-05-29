#ifndef PARAMETRICLINE_HPP
#define PARAMETRICLINE_HPP

#include <eigen3/Eigen/Dense>

struct ParametricLine {
    /// Ortsvektor
    Eigen::Array2d p;
    /// Richtungsvektor
    Eigen::Array2d u;

    Eigen::Array2d eval(double s) const;
    double solve_for_s(Eigen::Array2d const& query_point) const;

};

ParametricLine parametric_form(Eigen::Array2d const &p1, Eigen::Array2d const &p2);

#endif
