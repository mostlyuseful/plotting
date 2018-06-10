#include "line.hpp"

Eigen::Array3d homogenous_form(Eigen::Array2d const &p1,
                                      Eigen::Array2d const &p2) {
    Eigen::Vector3d const a(p1(0), p1(1), 1.0);
    Eigen::Vector3d const b(p2(0), p2(1), 1.0);
    Eigen::Vector3d const c = a.cross(b);
    return c;
}


Line Line::from_endpoints(Eigen::Array2d const &p1,
                                  Eigen::Array2d const &p2) {
    return Line{homogenous_form(p1,p2), parametric_form(p1,p2), p1, p2};
}

Line Line::from_endpoints(double const x1, double const y1, double const x2, double const y2) {
    Eigen::Array2d p1(x1,y1);
    Eigen::Array2d p2(x2,y2);
    return Line::from_endpoints(p1, p2);
}

Slope Line::inv_slope() const {
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
