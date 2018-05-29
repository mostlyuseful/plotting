#ifndef RASTER_HPP
#define RASTER_HPP

#include "dpath.hpp"

#include <boost/optional.hpp>

#include <clipper.hpp>

struct Bounds {
    ClipperLib::cInt min_x, max_x, min_y, max_y;
};

Bounds bounds_path(ClipperLib::Path const &path);
Bounds bounds_extend(Bounds const &a, Bounds const &b);
boost::optional<Bounds> bounds_paths(ClipperLib::Paths const &paths);
ClipperLib::Paths generate_lines_pattern(Bounds const &bounds,
                                  double const dy);
ClipperLib::DPaths raster_polygon_eo(ClipperLib::DPaths const &polygon_paths,
                                     double dy);

#endif /* RASTER_HPP */
