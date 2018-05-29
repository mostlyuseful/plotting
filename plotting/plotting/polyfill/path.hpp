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

    static inline Path from_span(double const y, Span const span) {
        Eigen::ArrayXd xs(2);
        Eigen::ArrayXd ys(2);
        xs << span.x0, span.x1;
        ys << y, y;
        return Path{xs, ys};
    }

    static inline std::vector<Path> from_rasterLine(RasterLine const line) {
        std::vector<Path> paths;

        for (auto const &span : line.spans) {
            paths.emplace_back(Path::from_span(line.y, span));
        }
        return paths;
    }

    inline Eigen::Array2d start() const { return Eigen::Array2d(xs(0), ys(0)); }
    inline Eigen::Array2d end() const {
        return Eigen::Array2d(xs.tail(1)(0), ys.tail(1)(0));
    }
    inline Path reversed() const { return Path{xs.reverse(), ys.reverse()}; }
    inline Path append(Path const &tail) const {
        Eigen::ArrayXd expanded_xs(xs.size() + tail.xs.size());
        Eigen::ArrayXd expanded_ys(ys.size() + tail.ys.size());
        expanded_xs << xs, tail.xs;
        expanded_ys << ys, tail.ys;
        return Path{expanded_xs, expanded_ys};
    }
};

using Paths = std::vector<Path>;

#endif /* PATH_HPP */
