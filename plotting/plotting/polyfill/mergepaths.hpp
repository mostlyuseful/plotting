#include "polygon.hpp"

#include <clipper.hpp>

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


inline ClipperLib::Path PolygonToClipperPath(Polygon const& pgon, double blowUpFactor) {
    ClipperLib::Path out;
    for(Eigen::Index i = 0; i < pgon.xs.size(); ++i) {
        auto const x = static_cast<ClipperLib::cInt>(pgon.xs(i)*blowUpFactor);
        auto const y = static_cast<ClipperLib::cInt>(pgon.ys(i)*blowUpFactor);
        out.push_back({x,y});
    }
    return out;
}

template <typename TRangePaths>
inline std::vector<Path> merge_close_paths(
        Polygon const& srcPolygon,
        TRangePaths paths,
        double const max_merge_distance) {
    using namespace ranges;


    // ClipperLib uses integer math, we use floating-point.
    // Multiply our coordinates before using them in ClipperLib, a naive version of fixed-point math
    double const blowUpFactor = 1000;
    ClipperLib::Path srcPolygonPath = PolygonToClipperPath(srcPolygon, blowUpFactor);


    auto violatesPolygon = [&srcPolygonPath, blowUpFactor](Candidate const& c) -> bool {
        namespace CL = ClipperLib;
        CL::Clipper clipper;
        CL::PolyTree solution;

        auto const& P = c.p->end();
        auto const& Q = c.q->start();
        CL::Path const connectorLine { {static_cast<CL::cInt>(P(0)*blowUpFactor), static_cast<CL::cInt>(P(1)*blowUpFactor)},
                                       {static_cast<CL::cInt>(Q(0)*blowUpFactor), static_cast<CL::cInt>(Q(1)*blowUpFactor)} };

        clipper.AddPath(srcPolygonPath, CL::ptClip, true);
        clipper.AddPath(connectorLine, CL::ptSubject, false);
        bool const ok = clipper.Execute(CL::ctDifference, solution, CL::pftEvenOdd, CL::pftEvenOdd);
        // If clipping fails, just pretend everything is okay :)
        if(!ok){
            return false;
        }
        // If something remains after clipping, they are lying outside the polygon.
        return (solution.Total() != 0);
    };


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

                std::vector<Candidate> const all_candidates {
                    make_candidate(p, q, q), make_candidate(p, std::make_shared<Path>(q->reversed()), q),
                    make_candidate(std::make_shared<Path>(p->reversed()), q, q),
                    make_candidate(std::make_shared<Path>(p->reversed()), std::make_shared<Path>(q->reversed()), q)
                };

                std::vector<Candidate> inside_candidates;
                for(Candidate const &c : all_candidates) {
                    if (!violatesPolygon(c)) {
                        inside_candidates.push_back(c);
                    }
                }

                if(!inside_candidates.empty()) {
                    auto it = std::min_element(
                        std::begin(inside_candidates), std::end(inside_candidates),
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