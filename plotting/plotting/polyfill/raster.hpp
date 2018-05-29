#ifndef RASTER_HPP
#define RASTER_HPP

#include "blow_up.hpp"
#include "dpath.hpp"
#include "polygon.hpp"

#include <boost/optional.hpp>
#include <iostream>
#include <limits>
#include <numeric>

struct Bounds {
    ClipperLib::cInt min_x, max_x, min_y, max_y;
};

inline Bounds bounds_path(ClipperLib::Path const &path) {
    using ClipperLib::cInt;
    cInt const min_x = std::accumulate(
        path.cbegin(), path.cend(), std::numeric_limits<cInt>::max(),
        [](cInt const accu, ClipperLib::IntPoint const &val) {
            return std::min(accu, val.X);
        });
    cInt const max_x = std::accumulate(
        path.cbegin(), path.cend(), std::numeric_limits<cInt>::min(),
        [](cInt const accu, ClipperLib::IntPoint const &val) {
            return std::max(accu, val.X);
        });
    cInt const min_y = std::accumulate(
        path.cbegin(), path.cend(), std::numeric_limits<cInt>::max(),
        [](cInt const accu, ClipperLib::IntPoint const &val) {
            return std::min(accu, val.Y);
        });
    cInt const max_y = std::accumulate(
        path.cbegin(), path.cend(), std::numeric_limits<cInt>::min(),
        [](cInt const accu, ClipperLib::IntPoint const &val) {
            return std::max(accu, val.Y);
        });
    return Bounds{min_x, max_x, min_y, max_y};
}

inline Bounds bounds_extend(Bounds const &a, Bounds const &b) {
    auto const min_x = std::min(a.min_x, b.min_x);
    auto const max_x = std::max(a.max_x, b.max_x);
    auto const min_y = std::min(a.min_y, b.min_y);
    auto const max_y = std::max(a.max_y, b.max_y);
    return Bounds{min_x, max_x, min_y, max_y};
}

inline boost::optional<Bounds> bounds_paths(ClipperLib::Paths const &paths) {
    boost::optional<Bounds> empty_bounds;
    return std::accumulate(
        paths.cbegin(), paths.cend(), empty_bounds,
        [](boost::optional<Bounds> const &accu, auto const &p) {
            if (!accu) {
                return boost::make_optional(bounds_path(p));
            } else {
                return boost::make_optional(bounds_extend(*accu, bounds_path(p)));
            }
        });
}

inline ClipperLib::Paths generate_lines_pattern(Bounds const &bounds,
                                         double const dy) {
    ClipperLib::Paths out;
    for(double y=bounds.min_y; y<bounds.max_y; y+=dy){
        auto const iy = static_cast<ClipperLib::cInt>(y);
        ClipperLib::Path line = {{bounds.min_x, iy}, {bounds.max_x, iy}};
        out.emplace_back(line);
    }
    return out;
}

inline ClipperLib::DPaths raster_polygon_eo(ClipperLib::DPaths const &polygon_paths,
                                     double dy) {
    double const path_scale = 1000;
    double const blown_up_dy = dy * path_scale;
    auto const blown_up_poly_paths = blow_up(polygon_paths, path_scale);
    auto const blown_up_lines =
        generate_lines_pattern(*bounds_paths(blown_up_poly_paths), blown_up_dy);
    namespace CL = ClipperLib;
    CL::Clipper clipper;
    CL::PolyTree solution;
    clipper.AddPaths(blown_up_poly_paths, CL::ptClip, true);
    clipper.AddPaths(blown_up_lines, CL::ptSubject, false);
    bool const ok = clipper.Execute(CL::ctIntersection, solution,
                                    CL::pftEvenOdd, CL::pftEvenOdd);

    if (!ok) {
        return {};
    }

    CL::Paths blown_up_out;
    CL::PolyTreeToPaths(solution, blown_up_out);

    CL::DPaths out_paths;
    out_paths.reserve(blown_up_out.size());
    for (auto const &path : blown_up_out) {
        CL::DPath out_path;
        out_path.reserve(path.size());
        for (auto const &pt : path) {
            double const x = pt.X / path_scale;
            double const y = pt.Y / path_scale;
            out_path.emplace_back(x,y);
        }
        out_paths.emplace_back(out_path);
    }

    return out_paths;
}

#endif /* RASTER_HPP */
