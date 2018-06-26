#ifndef MERGER_RTREE_HPP
#define MERGER_RTREE_HPP

#include "dpath.hpp"
#include "path.hpp"

#include "pimpl_h.hpp"

#include <vector>

class EndpointBasedMerger {
public:
    EndpointBasedMerger(ClipperLib::DPaths const &srcPolygon,
                        std::vector<Path> const &paths,
                        double const max_merge_distance);

    ~EndpointBasedMerger();

    std::vector<Path> merge();

private:
    class impl;
    pimpl<impl> m;
};

#endif // MERGER_RTREE_HPP
