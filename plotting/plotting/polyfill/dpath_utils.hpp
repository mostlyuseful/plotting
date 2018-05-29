#ifndef DPATH_UTILS_HPP
#define DPATH_UTILS_HPP

#include "dpath.hpp"
#include "path.hpp"
#include <enumerate.hpp>

inline Path dpath_to_path(ClipperLib::DPath const& dpath) {
    Eigen::ArrayXd xx(dpath.size());
    Eigen::ArrayXd yy(dpath.size());
    for(auto&& [i,xy_tup] : iter::enumerate(dpath)) {
        xx(i) = std::get<0>(xy_tup);
        yy(i) = std::get<1>(xy_tup);
    }
    return Path{xx,yy};
}

inline Paths dpaths_to_paths(ClipperLib::DPaths const& dpaths) {
    Paths out;
    out.reserve(dpaths.size());
    for(auto const& p:dpaths) {
        out.emplace_back(dpath_to_path(p));
    }
    return out;
}

#endif // DPATH_UTILS_HPP
