#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "edge.hpp"
#include "line.hpp"
#include "rasterline.hpp"

#include <eigen3/Eigen/Dense>

class Polygon {
    public:
    Eigen::ArrayXd xs;
    Eigen::ArrayXd ys;

    Polygon(Eigen::ArrayXd x, Eigen::ArrayXd y);
    double ymin() const;
    double ymax() const;
    std::vector<Line> lines() const;
    Polygon reversed() const;

    std::vector<Edge> sort_edges() const;

    std::vector<Span> get_spans(double const y) const;

    std::vector<RasterLine> raster(double dy = 1) const;
};

#endif
