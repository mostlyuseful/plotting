#ifndef SIMPLEAPI_HPP
#define SIMPLEAPI_HPP

#include "polygon.hpp"
#include "mergepaths.hpp"

#include <iostream>
#include <vector>

inline std::vector<RasterLine>
raster_polygon(std::vector<double> xx, std::vector<double> yy, double dy) {
    auto &&xs =
        Eigen::Map<Eigen::ArrayXd>(xx.data(), static_cast<Eigen::Index>(xx.size()));
    auto &&ys = Eigen::Map<Eigen::ArrayXd>(
        yy.data(), static_cast<Eigen::Index>(yy.size()));
    Polygon pgon(xs, ys);
    return pgon.raster(dy);
}

inline std::vector<Path>
raster_merge_polygon(std::vector<double> xx, std::vector<double> yy, double dy, double max_merge_distance) {
    auto &&xs = Eigen::Map<Eigen::ArrayXd>(xx.data(), static_cast<Eigen::Index>(xx.size()));
    auto &&ys = Eigen::Map<Eigen::ArrayXd>(yy.data(), static_cast<Eigen::Index>(yy.size()));
    Polygon pgon(xs, ys);
    std::vector<RasterLine> lines = pgon.raster(dy);
    std::vector<Path> paths = paths_from_raster_lines(lines);
    std::vector<Path> merged_paths = merge_close_paths(pgon, paths, max_merge_distance);
    return merged_paths;
}

#endif /* SIMPLEAPI_HPP */
