#include "parametricline.hpp"

ParametricLine parametric_form(Eigen::Array2d const &p1,
                                      Eigen::Array2d const &p2) {
    // Directional vector
    Eigen::Array2d const u(p2(0) - p1(0), p2(1) - p1(1));
    // Positional vector
    Eigen::Array2d const p = p1;
    return ParametricLine{p, u};
}


Eigen::Array2d ParametricLine::eval(double s) const {
    return p + s*u;
}

double ParametricLine::solve_for_s(Eigen::Array2d const& query_point) const {
    auto const dx = u(0);
    auto const dy = u(1);
    if(std::abs(dx) > std::abs(dy)) {
        auto const query_x = query_point(0);
        auto const foot_x = p(0);
        // query_x = foot_x + s * dx
        // query_x - foot_x = s * dx
        // (query_x - foot_x) / dx = s
        auto const s = (query_x - foot_x) / dx;
        return s;
    }else{
        auto const query_y = query_point(1);
        auto const foot_y = p(1);
        // query_y = foot_y + s * dy
        // query_y - foot_y = s * dy
        // (query_y - foot_y) / dy = s
        auto const s = (query_y - foot_y) / dy;
        return s;
    }
}

/*double ParametricLine::solve_for_s(Eigen::Array2d const& query_point) const {
    Eigen::Matrix<double, 2,1> a = u.matrix().eval();
    Eigen::Matrix<double, 2,1> b = query_point-p;
    auto const result = a.colPivHouseholderQr().solve(b).eval();
    return result(0,0);
}*/

