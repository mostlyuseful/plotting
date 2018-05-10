#ifndef EDGE_HPP
#define EDGE_HPP

#include "line.hpp"
#include "slope.hpp"

#include <boost/optional.hpp>
#include <eigen3/Eigen/Dense>

struct Edge {
    Line line;
    double ymin;
    double ymax;
    Slope slope;
    boost::optional<Eigen::Array2d> last_intersection_point;

    inline Edge(Line line, double ymin, double ymax, Slope slope,
                boost::optional<Eigen::Array2d> last_intersection_point)
        : line(std::move(line)), ymin(ymin), ymax(ymax), slope(slope),
          last_intersection_point(last_intersection_point) {}

    static inline Edge from_line(Line line) {
        auto const ymin = std::min(line.p1.y(), line.p2.y());
        auto const ymax = std::max(line.p1.y(), line.p2.y());
        return Edge{std::move(line), ymin, ymax, line.inv_slope(), {}};
    }
};

#endif
