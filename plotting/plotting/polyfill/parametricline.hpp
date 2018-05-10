#ifndef PARAMETRICLINE_HPP
#define PARAMETRICLINE_HPP

#include <eigen3/Eigen/Dense>

struct ParametricLine {
    /// Ortsvektor
    Eigen::Array2d p;
    /// Richtungsvektor
    Eigen::Array2d u;

    inline Eigen::Array2d eval(double s) const {
        return p + s*u;
    }

    inline double solve_for_s(Eigen::Array2d const& query_point) const {
        Eigen::Matrix<double, 2,1> a = u.matrix().eval();
        Eigen::Matrix<double, 2,1> b = query_point-p;
        auto const result = a.colPivHouseholderQr().solve(b).eval();
        return result(0,0);
    }

};

inline ParametricLine parametric_form(Eigen::Array2d const &p1,
                                      Eigen::Array2d const &p2) {
    // Directional vector
    Eigen::Array2d const u(p2(0) - p1(0), p2(1) - p1(1));
    // Positional vector
    Eigen::Array2d const p = p1;
    return ParametricLine{p, u};
}

#endif