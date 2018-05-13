#include "polygon.hpp"

#include <range/v3/core.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/transform.hpp>

#include <iostream>
#include <limits>
#include <list>
#include <memory>

/**
 * A path is different from a polygon in that it is simply a dumb holder of
 * linked coordinates
 */
struct Path {
    Eigen::ArrayXd xs;
    Eigen::ArrayXd ys;

    static inline Path from_span(double const y, Span const span) {
        Eigen::ArrayXd xs(2);
        Eigen::ArrayXd ys(2);
        xs << span.x0, span.x1;
        ys << y, y;
        return Path{xs, ys};
    }

    static inline std::vector<Path> from_rasterLine(RasterLine const line) {
        using namespace ranges;

        std::vector<Path> paths;

        for(auto const& span: line.spans) {
            paths.emplace_back(Path::from_span(line.y, span));
        }
        return paths;
    }

    inline Eigen::Array2d start() const { return Eigen::Array2d(xs(0), ys(0)); }
    inline Eigen::Array2d end() const {
        return Eigen::Array2d(xs.tail(1)(0), ys.tail(1)(0));
    }
    inline Path reversed() const {
        return Path{xs.reverse(), ys.reverse()};
    }
    inline Path append(Path const& tail) const {
        Eigen::ArrayXd expanded_xs(xs.size() + tail.xs.size());
        Eigen::ArrayXd expanded_ys(ys.size() + tail.ys.size());
        expanded_xs << xs, tail.xs;
        expanded_ys << ys, tail.ys;
        return Path{expanded_xs, expanded_ys};
    }
};

template <typename TRangeRasterLines>
inline std::vector<Path>
paths_from_raster_lines(TRangeRasterLines rasterLines) {
    using namespace ranges;
    std::vector<Path> paths;
    for(auto const& line : rasterLines) {
        for(auto&& path : Path::from_rasterLine(line)) {
            paths.emplace_back(path);
        }
    }
    return paths;
}

struct Candidate {
    double dst;
    std::shared_ptr<Path> p;
    std::shared_ptr<Path> q;
    std::shared_ptr<Path> q_orig;
};

inline Candidate make_candidate(std::shared_ptr<Path> p,
                                std::shared_ptr<Path> q,
                                std::shared_ptr<Path> q_orig) {
    auto const dst = (p->end() - q->start()).matrix().norm();
    return {dst, p, q, q_orig};
}

template <typename TRangePaths>
inline std::vector<Path> merge_close_paths(TRangePaths paths,
                                           double const max_merge_distance) {
    using namespace ranges;
    std::list<std::shared_ptr<Path>> pool =
        paths | view::transform([](Path const &p) {
            return std::make_shared<Path>(p);
        }) |
        to_<std::list>();
    while (true) {
        bool merged_at_least_one = false;
        for (auto const &p : pool) {
            double min_dst = std::numeric_limits<double>::infinity();
            std::shared_ptr<Path> min_p;
            std::shared_ptr<Path> min_q;
            std::shared_ptr<Path> min_q_orig;
            for (auto const &q : pool) {
                if (q == p) {
                    continue;
                }

                std::array<Candidate, 4> const candidates{
                    make_candidate(p, q, q), make_candidate(p, std::make_shared<Path>(q->reversed()), q),
                    make_candidate(std::make_shared<Path>(p->reversed()), q, q),
                    make_candidate(std::make_shared<Path>(p->reversed()), std::make_shared<Path>(q->reversed()), q)};

                auto it = std::min_element(
                    std::begin(candidates), std::end(candidates),
                    [](Candidate const &a, Candidate const &b) {
                        return a.dst < b.dst;
                    });
                
                if(it->dst < min_dst) {
                    min_dst = it->dst;
                    min_p = it->p;
                    min_q = it->q;
                    min_q_orig = it->q_orig;
                }
            }

            if ((min_dst < max_merge_distance) && (min_q)) {
                pool.erase(std::find(pool.begin(), pool.end(), p));
                pool.erase(std::find(pool.begin(), pool.end(), min_q_orig));
                pool.push_back(std::make_shared<Path>(min_p->append(*min_q)));
                merged_at_least_one = true;
                break;
            }
        }
        if(!merged_at_least_one){break;}
    }
    return pool | view::transform([](auto p){return *p;}) | to_<std::vector>();
}
