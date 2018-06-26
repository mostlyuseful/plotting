#ifndef MERGEPATHS_HPP
#define MERGEPATHS_HPP

#include "candidate.hpp"
#include "dpath.hpp"
#include "path.hpp"
#include "polygon.hpp"

#include <clipper.hpp>
#include <eigen3/Eigen/Dense>

#include <memory>
#include <vector>

template <typename TRangeRasterLines>
inline std::vector<Path>
paths_from_raster_lines(TRangeRasterLines rasterLines) {
    std::vector<Path> paths;
    for (auto const &line : rasterLines) {
        for (auto &&path : Path::from_rasterLine(line)) {
            paths.emplace_back(path);
        }
    }
    return paths;
}

double dst(Eigen::Array2d const& a, Eigen::Array2d const& b);

ClipperLib::Path PolygonToClipperPath(Polygon const &pgon, double blowUpFactor);

double cordLength(ClipperLib::Path const& path);

std::vector<Path> merge_close_paths_eo(ClipperLib::DPaths const &srcPolygon,
                     std::vector<Path> const &paths,
                     double const max_merge_distance);

#endif /* MERGEPATHS_HPP */
