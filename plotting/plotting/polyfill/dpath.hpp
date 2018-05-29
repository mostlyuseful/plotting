#ifndef DPATH_HPP
#define DPATH_HPP

#include <tuple>
#include <vector>

namespace ClipperLib {
using DPath = std::vector<std::tuple<double, double>>;
using DPaths = std::vector<DPath>;
} // namespace ClipperLib

#endif /* DPATH_HPP */
