#include "path.hpp"

Path Path::from_span(double const y, Span const span) {
    Eigen::ArrayXd xs(2);
    Eigen::ArrayXd ys(2);
    xs << span.x0, span.x1;
    ys << y, y;
    return Path{xs, ys};
}

std::vector<Path> Path::from_rasterLine(RasterLine const line) {
    std::vector<Path> paths;

    for (auto const &span : line.spans) {
        paths.emplace_back(Path::from_span(line.y, span));
    }
    return paths;
}

Eigen::Array2d Path::start() const { return Eigen::Array2d(xs(0), ys(0)); }
Eigen::Array2d Path::end() const {
    return Eigen::Array2d(xs.tail(1)(0), ys.tail(1)(0));
}
Path Path::reversed() const { return Path{xs.reverse(), ys.reverse()}; }
Path Path::append(Path const &tail) const {
    Eigen::ArrayXd expanded_xs(xs.size() + tail.xs.size());
    Eigen::ArrayXd expanded_ys(ys.size() + tail.ys.size());
    expanded_xs << xs, tail.xs;
    expanded_ys << ys, tail.ys;
    return Path{expanded_xs, expanded_ys};
}


