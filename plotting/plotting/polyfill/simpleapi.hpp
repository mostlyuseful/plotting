#ifndef SIMPLEAPI_HPP
#define SIMPLEAPI_HPP

#include "dpath.hpp"
#include "dpath_utils.hpp"
#include "polygon.hpp"
#include "mergepaths.hpp"
#include "raster.hpp"

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

inline std::vector<Path>
raster_merge_polygon_eo(ClipperLib::DPaths polygon_paths,
                        double dy,
                        double max_merge_distance) {
    std::cout << "START: raster_merge_polygon_eo" << std::endl;
    ClipperLib::DPaths raster_dpaths = raster_polygon_eo(polygon_paths, dy);
    std::cout << "AFTER: raster_polygon_eo" << std::endl;
    std::vector<Path> raster_paths = dpaths_to_paths(raster_dpaths);
    std::cout << "AFTER: dpaths_to_paths" << std::endl;
    auto merged_paths = merge_close_paths_eo(polygon_paths, raster_paths, max_merge_distance);
    std::cout << "AFTER: merge_close_paths_eo" << std::endl;
    return merged_paths;
}

#endif /* SIMPLEAPI_HPP */
