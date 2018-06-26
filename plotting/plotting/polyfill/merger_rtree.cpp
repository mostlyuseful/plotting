#include "merger_rtree.hpp"
#include "blow_up.hpp"
#include "pimpl_impl.hpp"
#include "pnpoly_rtree.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/join.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <tuple>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

enum class PointType : uint8_t { Start, End };

struct PathCandidate {
  PointType type;
  std::shared_ptr<std::optional<Path>> path;

  bool is_valid() const { return static_cast<bool>(path) && path->has_value(); }

  void invalidate() {
    if (!is_valid()) {
      throw std::logic_error("");
    }
    *path = {};
  }
};

using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using Box = bg::model::box<Point>;
using Value = std::pair<Point, PathCandidate>;
using Rtree = bgi::rtree<Value, bgi::quadratic<4>>;

namespace {
double sqrDistance(Point const a, Point const b) {
  double const dx = a.get<0>() - b.get<0>();
  double const dy = a.get<1>() - b.get<1>();
  return (dx * dx) + (dy * dy);
}
double distance(Point const a, Point const b) {
  return std::sqrt(sqrDistance(a, b));
}

// ClipperLib uses integer math, we use floating-point.
// Multiply our coordinates before using them in ClipperLib, a naive
// version of fixed-point math
double constexpr blowUpFactor = 1000;

} // namespace

class EndpointBasedMerger::impl {
public:
  impl() {}

  Rtree rtree;
  pip::rtree::MemoizedInclusionTester clippingTester;
  double max_merge_distance;

  std::vector<Value> queryValidByDistance(Point const center,
                                          double const radius) const {
    Point min_corner(center.get<0>() - radius, center.get<1>() - radius);
    Point max_corner(center.get<0>() + radius, center.get<1>() + radius);
    Box searchBox(min_corner, max_corner);
    auto rng = this->rtree |
               bgi::adaptors::queried(bgi::intersects(searchBox) &&
                                      bgi::satisfies([](Value const &value) {
                                        return value.second.is_valid();
                                      }));
    std::vector<Value> out;
    boost::copy(rng, std::back_inserter(out));
    std::sort(out.begin(), out.end(), [center](Value const &a, Value const &b) {
      double const dst_a = sqrDistance(a.first, center);
      double const dst_b = sqrDistance(b.first, center);
      return dst_a < dst_b;
    });
    return out;
  }

  bool violatesPolygon(Point const p, Point const q) {
    namespace CL = ClipperLib;
    double const x1 = p.get<0>() * blowUpFactor;
    double const y1 = p.get<1>() * blowUpFactor;
    double const x2 = q.get<0>() * blowUpFactor;
    double const y2 = q.get<1>() * blowUpFactor;
    Line const connectorLine = Line::from_endpoints(x1, y1, x2, y2);
    return !(this->clippingTester.contains(connectorLine));
  }

  bool isPathUnobstructed(Value const &a, Value const &b) {
    bool obstructed = this->violatesPolygon(a.first, b.first);
    return !obstructed;
  }
};

namespace {

template <typename TRangeOfPath> Rtree setup_rtree(TRangeOfPath paths) {
  std::vector<Value> startAndEndpoints;
  for (auto const &p : paths) {
    auto const sharedPath =
        std::make_shared<std::optional<Path>>(std::make_optional<Path>(p));
    Point startPt(p.start()(0), p.start()(1));
    Point endPt(p.end()(0), p.end()(1));
    startAndEndpoints.push_back(
        Value{startPt, PathCandidate{PointType::Start, sharedPath}});
    startAndEndpoints.push_back(
        Value{endPt, PathCandidate{PointType::End, sharedPath}});
  }
  return Rtree(startAndEndpoints);
}

pip::rtree::MemoizedInclusionTester
setup_clipping(ClipperLib::DPaths const &srcPolygon) {
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
  return inclusionTester;
}

std::optional<Value> chooseAnyValidValue(Rtree &rtree) {
  auto rng =
      rtree | bgi::adaptors::queried(bgi::satisfies(
                  [](Value const &value) { return value.second.is_valid(); }));
  if (boost::empty(rng)) {
    return {};
  } else {
    return std::make_optional(*boost::begin(rng));
  }
}

Path link(Value const &a, Value const &b) {
  if (!a.second.is_valid() || !b.second.is_valid()) {
    throw std::logic_error("Paths not valid");
  }
  Path const &pa = a.second.path->value();
  Path const &pb = b.second.path->value();

  if (a.second.type == PointType::Start) {
    if (b.second.type == PointType::Start) {
      return pa.reversed().append(pb);
    } else {
      // b.second.type==PointType::End
      return pb.append(pa);
    }
  } else {
    // a.second.type==PointType::End
    if (b.second.type == PointType::Start) {
      return pa.append(pb);
    } else {
      // b.second.type==PointType::End
      return pa.append(pb.reversed());
    }
  }
}

} // namespace

EndpointBasedMerger::EndpointBasedMerger(const ClipperLib::DPaths &srcPolygon,
                                         const std::vector<Path> &paths,
                                         const double max_merge_distance) {
  m->max_merge_distance = max_merge_distance;
  m->rtree = setup_rtree(paths);
  m->clippingTester = setup_clipping(srcPolygon);
}

EndpointBasedMerger::~EndpointBasedMerger() {}

std::vector<Path> EndpointBasedMerger::merge() {
  double const radius = m->max_merge_distance;

  while (true) {
    unsigned counter = 0;
    auto ov1 = chooseAnyValidValue(m->rtree);
    if (!ov1) {
      break;
    }
    auto &v1 = *ov1;
    for (auto &v2 : m->queryValidByDistance(v1.first, radius)) {
      bool const differentValue = v1.second.path != v2.second.path;
      if (differentValue && m->isPathUnobstructed(v1, v2)) {
        auto linked = std::make_shared<std::optional<Path>>(link(v1, v2));
        v1.second.invalidate();
        v2.second.invalidate();
        // Value{startPt, PathCandidate{PointType::Start, sharedPath}}
        m->rtree.insert(
            Value{Point(linked->value().start()(0), linked->value().start()(1)),
                  {PointType::Start, linked}});
        m->rtree.insert(
            Value{Point(linked->value().end()(0), linked->value().end()(1)),
                  {PointType::End, linked}});
        counter++;
        break;
      }
    }
    if (counter == 0) {
      break;
    }
  }
  std::vector<Path> merged;
  for (auto const &p :
       m->rtree | bgi::adaptors::queried(bgi::satisfies([](Value const &value) {
         return value.second.type == PointType::Start &&
                value.second.is_valid();
       }))) {
    merged.push_back(*(*p.second.path));
  }
  return merged;
}
