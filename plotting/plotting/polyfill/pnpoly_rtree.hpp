#ifndef PNPOLY_RTREE_HPP
#define PNPOLY_RTREE_HPP

#include "intersection.hpp"
#include "line.hpp"

#include <range/v3/core.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/drop_exactly.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <clipper.hpp>

#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <array>
#include <map>
#include <tuple>

namespace pip {
namespace rtree {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using Box = bg::model::box<Point>;
using Value = std::pair<Box, Line>;

inline Box
return_envelope(std::tuple<ClipperLib::IntPoint, ClipperLib::IntPoint> points) {
  auto const &p = std::get<0>(points);
  auto const &q = std::get<1>(points);
  double const min_x = std::min(p.X, q.X);
  double const max_x = std::max(p.X, q.X);
  double const min_y = std::min(p.Y, q.Y);
  double const max_y = std::max(p.Y, q.Y);
  Point const min_corner(min_x, min_y);
  Point const max_corner(max_x, max_y);
  return Box(min_corner, max_corner);
}

inline auto iter_segments(ClipperLib::Path const &path) {
  auto rngA = ranges::view::take_exactly(path, path.size() - 1);
  auto rngB = ranges::view::drop_exactly(path, 1);
  return ranges::view::zip(rngA, rngB);
}

inline bool even_odd_rule_inside(unsigned int const num_crossings) {
  return (num_crossings % 2) == 1;
}

inline Box dilate_bounds(Box const &bounds, double const amount) {
  Point min_corner(bounds.min_corner().get<0>() - amount,
                   bounds.min_corner().get<1>() - amount);
  Point max_corner(bounds.max_corner().get<0>() + amount,
                   bounds.max_corner().get<1>() + amount);
  return Box(min_corner, max_corner);
}

inline Box make_box(Line const &line) {
  auto const x1 = line.p1(0);
  auto const y1 = line.p1(1);
  auto const x2 = line.p2(0);
  auto const y2 = line.p2(1);
  auto const min_x = std::min(x1, x2);
  auto const max_x = std::max(x1, x2);
  auto const min_y = std::min(y1, y2);
  auto const max_y = std::max(y1, y2);
  return Box(Point(min_x, min_y), Point(max_x, max_y));
}

class InclusionTester {
public:
  inline InclusionTester(ClipperLib::Paths const &paths) {
    auto rng = ranges::view::for_each(paths, [](ClipperLib::Path const &path) {
      return ranges::view::transform(iter_segments(path), [](auto two_points) {
        Box bbox = return_envelope(two_points);
        auto const &[p, q] = two_points;
        Line segment = Line::from_endpoints(p.X, p.Y, q.X, q.Y);
        Value val = std::make_pair(bbox, segment);
        return val;
      });
    });
    for (auto it : rng) {
      rtree.insert(it);
    }
  }

  inline bool outside_bbox(double const x, double const y) const {
    bool const outside_x = (x < rtree.bounds().min_corner().get<0>()) ||
                           (x > rtree.bounds().max_corner().get<0>());
    bool const outside_y = (y < rtree.bounds().min_corner().get<1>()) ||
                           (y > rtree.bounds().max_corner().get<1>());
    return outside_x || outside_y;
  }

  inline std::vector<Line> query_inside_box(Box const &box) const {
    std::vector<Line> lines;
    for (Value const &v :
         rtree | bgi::adaptors::queried(bgi::intersects(box))) {
      lines.push_back(v.second);
    }
    return lines;
  }

  inline std::vector<Line> query_all_to_right(double const x, double const y,
                                              Box const &bbox) const {
    double const min_x = x;
    double const max_x = bbox.max_corner().get<0>();
    double const min_y = y;
    double const max_y = y;
    Box box_region(Point(min_x, min_y), Point(max_x, max_y));
    return query_inside_box(box_region);
  }

  inline Line make_sweeping_ray(double const min_x, double const y,
                                Box const &bbox) const {
    auto const max_x = bbox.max_corner().get<0>();
    return Line::from_endpoints(min_x, y, max_x, y);
  }

  inline bool contains(double const x, double const y) const {
    if (outside_bbox(x, y)) {
      return false;
    }
    auto const eps = 1.0;
    auto const bbox = dilate_bounds(rtree.bounds(), eps);
    auto const intersecting_segments = query_all_to_right(x, y, bbox);
    auto const sweeping_ray = make_sweeping_ray(x, y, bbox);
    auto const intersections =
        intersecting_segments |
        ranges::view::transform([&sweeping_ray](auto const &s) {
          return intersect_lines(s, sweeping_ray);
        }) |
        ranges::to_vector;

    auto const crossings =
        intersections | ranges::view::filter([](auto const &variant) {
          IntersectionPoint const *p = boost::get<IntersectionPoint>(&variant);
          return p != nullptr;
        }) |
        ranges::view::transform([](auto const &variant) {
          return boost::get<IntersectionPoint>(variant);
        }) |
        ranges::view::filter([](auto const &ix) { return ix.on_l2; }) |
        ranges::view::transform([](auto const &ix) { return ix.s2; }) |
        ranges::to_<std::set>();
    return even_odd_rule_inside(crossings.size());
  }

  inline bool contains(Point const &point) const {
    return this->contains(point.get<0>(), point.get<1>());
  }

  inline bool contains(Eigen::Array2d const &point) const {
    return this->contains(point(0), point(1));
  }

  inline bool contains(Line const &line) {
    // Take a line where both endpoints lie exactly on an edge.
    // A line connecting these endpoints would be detected as in-polygon
    // when there is only empty space between the endpoints since it does
    // not cross an edge and both points are in-polygon...
    Eigen::Array2d const midpoint = (line.p1 + line.p2) / 2;
    if ((!this->contains(line.p1)) || (!this->contains(line.p2)) ||
        (!this->contains(midpoint))) {
      return false;
    }
    auto const intersecting_segments = this->query_inside_box(make_box(line));
    for (auto const &segment : intersecting_segments) {
      auto const intersection = intersect_lines(segment, line);
      {
        DenormalIntersection const *p =
            boost::get<DenormalIntersection>(&intersection);
        if (p != nullptr && *p == DenormalIntersection::everywhere) {
          return false;
        }
      }
      {
        IntersectionPoint const *p =
            boost::get<IntersectionPoint>(&intersection);
        if (p != nullptr) {
          if (p->on_l1 || p->on_l2) {
            return false;
          }
        }
      }
    }
    return true;
  }

protected:
  bgi::rtree<Value, bgi::quadratic<16>> rtree;
};

class MemoizedInclusionTester {
public:
  using PointTuple = std::tuple<double, double>;
  using LineEndpoints = std::tuple<PointTuple, PointTuple>;
  MemoizedInclusionTester(ClipperLib::Paths const &paths) : m_tester(paths) {}

  inline bool contains(double const x, double const y) {
    PointTuple const point = std::make_tuple(x, y);
    auto it = m_memo_points.find(point);
    if (it != m_memo_points.end()) {
      return it->second;
    } else {
      bool const contained = m_tester.contains(x, y);
      m_memo_points[point] = contained;
      return contained;
    }
  }
  inline bool contains(Point const &point) {
    return this->contains(point.get<0>(), point.get<1>());
  }
  inline bool contains(Eigen::Array2d const &point) {
    return this->contains(point(0), point(1));
  }
  inline bool contains(Line const &line) {
    auto const ep = endpoints(line);
    auto it = m_memo_lines.find(ep);
    if (it != m_memo_lines.end()) {
      return it->second;
    } else {
      bool const contained = m_tester.contains(line);
      m_memo_lines[ep] = contained;
      return contained;
    }
  }

protected:
  inline static LineEndpoints endpoints(Line const &line) {
    PointTuple const from = std::make_tuple(line.p1(0), line.p1(1));
    PointTuple const to = std::make_tuple(line.p2(0), line.p2(1));
    return std::make_tuple(from, to);
  }

protected:
  InclusionTester m_tester;
  std::map<PointTuple, bool> m_memo_points;
  std::map<LineEndpoints, bool> m_memo_lines;
};

} // namespace rtree
} // namespace pip

#endif // PNPOLY_RTREE_HPP
