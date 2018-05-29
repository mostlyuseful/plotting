#ifndef DPATH_UTILS_HPP
#define DPATH_UTILS_HPP

#include "dpath.hpp"
#include "path.hpp"

Path dpath_to_path(ClipperLib::DPath const& dpath);
Paths dpaths_to_paths(ClipperLib::DPaths const& dpaths);

#endif // DPATH_UTILS_HPP
