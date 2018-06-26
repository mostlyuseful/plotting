#include "mergepaths.hpp"
#include "blow_up.hpp"
#include "merger_rtree.hpp"
#include "path.hpp"
#include "polygon.hpp"
#include "pnpoly_rtree.hpp"

#include <sliding_window.hpp>

#include <clipper.hpp>

#include <range/v3/core.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/indices.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <deque>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <x86intrin.h>

double dst(Eigen::Array2d const &a, Eigen::Array2d const &b) {
  return (a - b).matrix().norm();
}

ClipperLib::Path PolygonToClipperPath(Polygon const &pgon,
                                      double blowUpFactor) {
  ClipperLib::Path out;
  for (Eigen::Index i = 0; i < pgon.xs.size(); ++i) {
    auto const x = static_cast<ClipperLib::cInt>(pgon.xs(i) * blowUpFactor);
    auto const y = static_cast<ClipperLib::cInt>(pgon.ys(i) * blowUpFactor);
    out.push_back({x, y});
  }
  return out;
}

double cordLength(ClipperLib::Path const &path) {
  double length = 0;
  for (auto &&window : iter::sliding_window(path, 2)) {
    auto const &p = window.at(0);
    auto const &q = window.at(1);
    double const dx = q.X - p.X;
    double const dy = q.Y - p.Y;
    double const dst = std::hypot(dx, dy);
    length += dst;
  }
  return length;
}

class PnPolyVertices {
public:
  std::vector<float> x;
  std::vector<float> y;
};

PnPolyVertices ClipperPathsToPnPoly(ClipperLib::Paths const &paths) {
  PnPolyVertices out;
  for (auto const &path : paths) {
    out.x.push_back(0);
    out.y.push_back(0);
    for (auto const &pt : path) {
      out.x.push_back(pt.X);
      out.y.push_back(pt.Y);
    }
    auto const &first = path.front();
    auto const &last = path.back();
    if (first != last) {
      out.x.push_back(first.X);
      out.y.push_back(first.Y);
    }
  }
  out.x.push_back(0);
  out.y.push_back(0);

  return out;
}

struct Point {
  double x;
  double y;
};

std::vector<Point> sample_line(ClipperLib::Path const &line,
                               int const num_samples) {
  auto const &p = line[0];
  auto const &q = line[1];
  double const dx = q.X - p.X;
  double const dy = q.Y - p.Y;
  std::vector<Point> points;
  points.reserve(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    double const frac = i * (1.0 / (num_samples - 1));
    double const x = frac * dx + p.X;
    double const y = frac * dy + p.Y;
    points.push_back({x, y});
  }
  return points;
}

std::vector<Path> merge_close_paths_eo(ClipperLib::DPaths const &srcPolygon,
                                       std::vector<Path> const &paths,
                                       double const max_merge_distance) {
  EndpointBasedMerger merger(srcPolygon, paths, max_merge_distance);
  return merger.merge();
} //*/

std::vector<Path> merge_close_paths_eo_(ClipperLib::DPaths const &srcPolygon,
                                        std::vector<Path> const &paths,
                                        double const max_merge_distance) {
  using namespace ranges;

  // ClipperLib uses integer math, we use floating-point.
  // Multiply our coordinates before using them in ClipperLib, a naive
  // version of fixed-point math
  double const blowUpFactor = 1000;
  ClipperLib::Paths blownUpPolygon = blow_up(srcPolygon, blowUpFactor);

  // Offset (dilate) src boundary polygon just slightly to allow connecting
  // lines which intersect with src boundary polygon's edges.
  auto offsetted = [&blownUpPolygon]() {
    double const offsetAmount = 1.0;
    ClipperLib::ClipperOffset co;
    ClipperLib::PolyTree solution;
    co.AddPaths(blownUpPolygon, ClipperLib::jtRound,
                ClipperLib::etClosedPolygon);
    co.Execute(solution, offsetAmount);
    ClipperLib::Paths out;
    ClipperLib::PolyTreeToPaths(solution, out);
    return out;
  }();

  pip::rtree::MemoizedInclusionTester inclusionTester(offsetted);

  auto violatesPolygon = [&inclusionTester,
                          blowUpFactor](Candidate const &c) -> bool {
    namespace CL = ClipperLib;
    auto const &P = c.p->end();
    auto const &Q = c.q->start();
    double const x1 = P(0) * blowUpFactor;
    double const y1 = P(1) * blowUpFactor;
    double const x2 = Q(0) * blowUpFactor;
    double const y2 = Q(1) * blowUpFactor;
    Line const connectorLine = Line::from_endpoints(x1, y1, x2, y2);
    return !inclusionTester.contains(connectorLine);
  };

  std::list<std::shared_ptr<Path>> pool =
      paths |
      view::transform([](Path const &p) { return std::make_shared<Path>(p); }) |
      to_<std::list>();

  auto radius_search = [](std::list<std::shared_ptr<Path>> const &pool,
                          std::shared_ptr<Path> p,
                          double const max_merge_distance) {
    auto const p1 = p->start();
    auto const p2 = p->end();

    std::vector<std::shared_ptr<Path>> out;
    for (auto const &q : pool) {
      auto const q1 = q->start();
      auto const q2 = q->end();
      bool const is_near = dst(p1, q1) < max_merge_distance ||
                           dst(p1, q2) < max_merge_distance ||
                           dst(p2, q1) < max_merge_distance ||
                           dst(p2, q2) < max_merge_distance;
      if (is_near) {
        out.push_back(q);
      }
    }
    // std::cout << "Found " << out.size() << " candidates" << std::endl;
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
              std::begin(inside_candidates), std::end(inside_candidates),
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
        // if(min_q){break;}
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
  return pool | view::transform([](auto p) { return *p; }) | to_<std::vector>();
}
//*/
