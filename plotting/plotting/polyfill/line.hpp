#ifndef LINE_HPP
#define LINE_HPP

#include "parametricline.hpp"
#include "slope.hpp"

#include <eigen3/Eigen/Dense>

inline Eigen::Array3d homogenous_form(Eigen::Array2d const &p1,
                                      Eigen::Array2d const &p2) {
    Eigen::Vector3d const a(p1(0), p1(1), 1.0);
    Eigen::Vector3d const b(p2(0), p2(1), 1.0);
    Eigen::Vector3d const c = a.cross(b);
    return c;
}

struct Line {
    Eigen::Array3d homogenous;
    ParametricLine parametric;
    Eigen::Array2d p1;
    Eigen::Array2d p2;

    static inline Line from_endpoints(Eigen::Array2d const &p1,
                                      Eigen::Array2d const &p2) {
        return Line{homogenous_form(p1,p2), parametric_form(p1,p2), p1, p2};
    }

    inline Slope inv_slope() const {
        auto const dy = p2.y() - p1.y();
        auto const dx = p2.x() - p1.x();
        if (dy == 0) {
            return HorizontalSlope();
        }

        if (dx == 0) {
            return FiniteSlope{0};
        } else {
            return FiniteSlope{dx / dy};
        }
    }
};

#endif