#ifndef PATH_HPP
#define PATH_HPP

#include "rasterline.hpp"

#include <eigen3/Eigen/Dense>
#include <vector>

/**
 * A path is different from a polygon in that it is simply a dumb holder of
 * linked coordinates
 */
struct Path {
    Eigen::ArrayXd xs;
    Eigen::ArrayXd ys;

    static Path from_span(double const y, Span const span);
    static std::vector<Path> from_rasterLine(RasterLine const line);

    Eigen::Array2d start() const;
    Eigen::Array2d end() const;
    Path reversed() const;
    Path append(Path const &tail) const;
};

using Paths = std::vector<Path>;

#endif /* PATH_HPP */
