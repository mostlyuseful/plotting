#ifndef MERGEPATHS_HPP
#define MERGEPATHS_HPP

#include "blow_up.hpp"
#include "path.hpp"
#include "polygon.hpp"

#include <sliding_window.hpp>

#include <clipper.hpp>

#include <range/v3/core.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/indices.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <numeric>

template <typename TRangeRasterLines>
inline std::vector<Path>
paths_from_raster_lines(TRangeRasterLines rasterLines) {
    using namespace ranges;
    std::vector<Path> paths;
    for (auto const &line : rasterLines) {
        for (auto &&path : Path::from_rasterLine(line)) {
            paths.emplace_back(path);
        }
    }
    return paths;
}

inline double dst(Eigen::Array2d const& a, Eigen::Array2d const& b) {
    return (a-b).matrix().norm();
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

inline ClipperLib::Path PolygonToClipperPath(Polygon const &pgon,
                                             double blowUpFactor) {
    ClipperLib::Path out;
    for (Eigen::Index i = 0; i < pgon.xs.size(); ++i) {
        auto const x = static_cast<ClipperLib::cInt>(pgon.xs(i) * blowUpFactor);
        auto const y = static_cast<ClipperLib::cInt>(pgon.ys(i) * blowUpFactor);
        out.push_back({x, y});
    }
    return out;
}

inline double cordLength(ClipperLib::Path const& path) {
    double length = 0;
    for(auto&& window : iter::sliding_window(path, 2)) {
        auto const& p = window.at(0);
        auto const& q = window.at(1);
        double const dx = q.X-p.X;
        double const dy = q.Y-p.Y;
        double const dst = std::hypot(dx,dy);
        length += dst;
    }
    return length;
}

inline std::vector<Path>
merge_close_paths_eo(ClipperLib::DPaths const &srcPolygon,
                     std::vector<Path> const &paths,
                     double const max_merge_distance) {
    using namespace ranges;

    // ClipperLib uses integer math, we use floating-point.
    // Multiply our coordinates before using them in ClipperLib, a naive
    // version of fixed-point math
    double const blowUpFactor = 1000;
    ClipperLib::Paths blownUpPolygon = blow_up(srcPolygon, blowUpFactor);

    // Offset (dilate) src boundary polygon just slightly to allow connecting lines which
    // intersect with src boundary polygon's edges.
    auto offsetted = [&blownUpPolygon]() {
        double const offsetAmount = 1.0;
        ClipperLib::ClipperOffset co;
        ClipperLib::PolyTree solution;
        co.AddPaths(blownUpPolygon, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
        co.Execute(solution, offsetAmount);
        ClipperLib::Paths out;
        ClipperLib::PolyTreeToPaths(solution, out);
        return out;
    }();

    auto n_points = [](ClipperLib::Paths const& paths) {
        return std::accumulate(paths.cbegin(), paths.cend(), 0, [](auto const& accu, ClipperLib::Path const& p){
            return accu + p.size();
        });
    };
    /*std::cerr << "Before cleaning: " << n_points(offsetted) << std::endl;
    ClipperLib::CleanPolygons(offsetted);
    std::cerr << "After cleaning: " << n_points(offsetted) << std::endl;
    //*/

    auto violatesPolygon = [&offsetted,
                            blowUpFactor](Candidate const &c) -> bool {
        namespace CL = ClipperLib;
        CL::Clipper clipper;
        CL::PolyTree solution;

        auto const &P = c.p->end();
        auto const &Q = c.q->start();
        CL::Path const connectorLine{
            {static_cast<CL::cInt>(P(0) * blowUpFactor),
             static_cast<CL::cInt>(P(1) * blowUpFactor)},
            {static_cast<CL::cInt>(Q(0) * blowUpFactor),
             static_cast<CL::cInt>(Q(1) * blowUpFactor)}};

        // clipper.AddPaths(blownUpPolygon, CL::ptClip, true);
        clipper.AddPaths(offsetted, CL::ptClip, true);
        clipper.AddPath(connectorLine, CL::ptSubject, false);
        bool const ok = clipper.Execute(CL::ctDifference, solution,
                                        CL::pftEvenOdd, CL::pftEvenOdd);
        // If clipping fails, just pretend everything is okay :)
        if (!ok) {
            std::cerr << "!OK" << std::endl;
            return false;
        }
        // If something remains after clipping, they are lying outside the polygon
        bool const somethingLiesOutside = (solution.Total() != 0);

        /*
        double const outsideCordLength = [&solution](){
            ClipperLib::Paths linePaths;
            ClipperLib::OpenPathsFromPolyTree(solution, linePaths);
            return std::accumulate(
                        linePaths.cbegin(), linePaths.cend(),
                        0.0,
                        [](double const accu, ClipperLib::Path const& path){
                return accu + cordLength(path);
            });
        }();

        std::cerr << "DEBUG: outsideCordLength = " << outsideCordLength << std::endl;
        //*/

        return somethingLiesOutside;
    };

    std::list<std::shared_ptr<Path>> pool =
        paths
            | view::transform([](Path const& p) {
                return std::make_shared<Path>(p);
              })
            | to_<std::list>();

    auto radius_search = [](std::list<std::shared_ptr<Path>> const& pool,
            std::shared_ptr<Path> p,
            double const max_merge_distance) {
        auto const p1 = p->start();
        auto const p2 = p->end();

        std::vector<std::shared_ptr<Path>> out;
        for (auto const& q : pool) {
            auto const q1 = q->start();
            auto const q2 = q->end();
            bool const is_near = dst(p1,q1)<max_merge_distance
                    || dst(p1,q2)<max_merge_distance
                    || dst(p2,q1)<max_merge_distance
                    || dst(p2,q2)<max_merge_distance;
            if(is_near) {
                out.push_back(q);
            }
        }
        return out;
    };

    while (true) {
        bool merged_at_least_one = false;
        for (auto const &p : pool) {
            double min_dst = std::numeric_limits<double>::infinity();
            std::shared_ptr<Path> min_p;
            std::shared_ptr<Path> min_q;
            std::shared_ptr<Path> min_q_orig;
            for (auto const &q : radius_search(pool, p, max_merge_distance)) {
                if (q == p) {
                    continue;
                }

                std::vector<Candidate> const all_candidates{
                    make_candidate(p, q, q),
                    make_candidate(p, std::make_shared<Path>(q->reversed()), q),
                    make_candidate(std::make_shared<Path>(p->reversed()), q, q),
                    make_candidate(std::make_shared<Path>(p->reversed()),
                                   std::make_shared<Path>(q->reversed()), q)};

                std::vector<Candidate> inside_candidates;
                for (Candidate const &c : all_candidates) {
                    if (!violatesPolygon(c)) {
                        inside_candidates.push_back(c);
                    }
                }

                if (!inside_candidates.empty()) {
                    auto it = std::min_element(
                        std::begin(inside_candidates),
                        std::end(inside_candidates),
                        [](Candidate const &a, Candidate const &b) {
                            return a.dst < b.dst;
                        });

                    if (it->dst < min_dst) {
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
        if (!merged_at_least_one) {
            break;
        }
    }
    return pool | view::transform([](auto p) { return *p; }) |
           to_<std::vector>();
}

template <typename TRangePaths>
inline std::vector<Path> merge_close_paths(Polygon const &srcPolygon,
                                           TRangePaths paths,
                                           double const max_merge_distance) {
    using namespace ranges;

    // ClipperLib uses integer math, we use floating-point.
    // Multiply our coordinates before using them in ClipperLib, a naive
    // version of fixed-point math
    double const blowUpFactor = 1000;
    ClipperLib::Path srcPolygonPath =
        PolygonToClipperPath(srcPolygon, blowUpFactor);

    auto violatesPolygon = [&srcPolygonPath,
                            blowUpFactor](Candidate const &c) -> bool {
        namespace CL = ClipperLib;
        CL::Clipper clipper;
        CL::PolyTree solution;

        auto const &P = c.p->end();
        auto const &Q = c.q->start();
        CL::Path const connectorLine{
            {static_cast<CL::cInt>(P(0) * blowUpFactor),
             static_cast<CL::cInt>(P(1) * blowUpFactor)},
            {static_cast<CL::cInt>(Q(0) * blowUpFactor),
             static_cast<CL::cInt>(Q(1) * blowUpFactor)}};

        clipper.AddPath(srcPolygonPath, CL::ptClip, true);
        clipper.AddPath(connectorLine, CL::ptSubject, false);
        bool const ok = clipper.Execute(CL::ctDifference, solution,
                                        CL::pftEvenOdd, CL::pftEvenOdd);
        // If clipping fails, just pretend everything is okay :)
        if (!ok) {
            return false;
        }
        // If something remains after clipping, they are lying outside the
        // polygon.
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

                std::vector<Candidate> const all_candidates{
                    make_candidate(p, q, q),
                    make_candidate(p, std::make_shared<Path>(q->reversed()), q),
                    make_candidate(std::make_shared<Path>(p->reversed()), q, q),
                    make_candidate(std::make_shared<Path>(p->reversed()),
                                   std::make_shared<Path>(q->reversed()), q)};

                std::vector<Candidate> inside_candidates;
                for (Candidate const &c : all_candidates) {
                    if (!violatesPolygon(c)) {
                        inside_candidates.push_back(c);
                    }
                }

                if (!inside_candidates.empty()) {
                    auto it = std::min_element(
                        std::begin(inside_candidates),
                        std::end(inside_candidates),
                        [](Candidate const &a, Candidate const &b) {
                            return a.dst < b.dst;
                        });

                    if (it->dst < min_dst) {
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
        if (!merged_at_least_one) {
            break;
        }
    }
    return pool | view::transform([](auto p) { return *p; }) |
           to_<std::vector>();
}

#endif /* MERGEPATHS_HPP */
