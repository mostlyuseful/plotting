#ifndef MERGECLOSEPATHS_HPP
#define MERGECLOSEPATHS_HPP

#include "candidate.hpp"
#include "mergepaths.hpp"
#include "path.hpp"
#include "polygon.hpp"


#include "clipper.hpp"

#include <range/v3/core.hpp>
#include <range/v3/view/transform.hpp>

#include <vector>

/*
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
            // TODO("Take first possible candidate without evaluating every candidate");
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
}//*/

#endif // MERGECLOSEPATHS_HPP
