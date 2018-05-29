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

    static Edge from_line(Line line);
};

#endif
