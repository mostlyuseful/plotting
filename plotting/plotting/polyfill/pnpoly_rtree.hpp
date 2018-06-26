#ifndef PNPOLY_RTREE_HPP
#define PNPOLY_RTREE_HPP

#include "intersection.hpp"
#include "line.hpp"

#include <clipper.hpp>

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

Box return_envelope(std::tuple<ClipperLib::IntPoint, ClipperLib::IntPoint> points);

bool even_odd_rule_inside(unsigned int const num_crossings);

Box dilate_bounds(Box const &bounds, double const amount);

Box make_box(Line const &line);

class InclusionTester {
public:
  InclusionTester();
  InclusionTester(ClipperLib::Paths const &paths);

  bool outside_bbox(double const x, double const y) const;

  std::vector<Line> query_inside_box(Box const &box) const;

  std::vector<Line> query_all_to_right(double const x, double const y,
                                              Box const &bbox) const;

  Line make_sweeping_ray(double const min_x, double const y,
                                Box const &bbox) const;

  bool contains(double const x, double const y) const;

  bool contains(Point const &point) const;

  bool contains(Eigen::Array2d const &point) const;

  bool contains(Line const &line) const;

protected:
  bgi::rtree<Value, bgi::quadratic<4>> rtree;
};

class MemoizedInclusionTester {
public:
  using PointTuple = std::tuple<double, double>;
  using LineEndpoints = std::tuple<PointTuple, PointTuple>;
  MemoizedInclusionTester();
  MemoizedInclusionTester(ClipperLib::Paths const &paths);

  bool contains(double const x, double const y);
  bool contains(Point const &point);
  bool contains(Eigen::Array2d const &point);
  bool contains(Line const &line);

protected:
  static LineEndpoints endpoints(Line const &line);

protected:
  InclusionTester m_tester;
  std::map<PointTuple, bool> m_memo_points;
  std::map<LineEndpoints, bool> m_memo_lines;
};

} // namespace rtree
} // namespace pip

#endif // PNPOLY_RTREE_HPP
