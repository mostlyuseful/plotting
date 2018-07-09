#include "sort_paths.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/join.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<double, 2, bg::cs::cartesian>;

bool operator==(Point const &lhs, Point const &rhs) {
  return (lhs.get<0>() == rhs.get<0>()) && (lhs.get<1>() == rhs.get<1>());
}

struct EndpointValue {
  size_t path_index;
  Point end;
};

bool operator==(EndpointValue const &lhs, EndpointValue const &rhs) {
  return (lhs.path_index == rhs.path_index) && (lhs.end == rhs.end);
}

using Value = std::pair<Point, EndpointValue>;
using Rtree = bgi::rtree<Value, bgi::quadratic<4>>;

namespace {

double get_x(Endpoint const &ep) { return std::get<0>(ep); }

double get_y(Endpoint const &ep) { return std::get<1>(ep); }

Rtree make_rtree(std::vector<PathEndpoints> endpoints) {

  auto rngIndexed = endpoints | boost::adaptors::indexed(0);
  auto f = [](decltype(rngIndexed)::value_type const &elem) {
    auto const &[ep_start, ep_end] = elem.value();
    Point start(get_x(ep_start), get_y(ep_start));
    Point end(get_x(ep_end), get_y(ep_end));
    Value v(start, {static_cast<std::size_t>(elem.index()), end});
    return v;
  };
  auto F =
      std::function<Value(decltype(rngIndexed)::value_type const &elem)>(f);

  auto rng = rngIndexed | boost::adaptors::transformed(F);
  Rtree rtree(rng);
  return rtree;
}

std::size_t get_index(Value const &v) { return v.second.path_index; }

std::optional<Value> find_follower(Value const &query, Rtree const &rtree) {
  auto rng = rtree | bgi::adaptors::queried(bgi::nearest(query.second.end, 1));
  for (auto const &v : rng) {
    return v;
  }
  return {};
}

Value find_most_top_left(Rtree &rtree) {

  auto const min_corner = rtree.bounds().min_corner();

  auto rng = rtree | bgi::adaptors::queried(bgi::nearest(min_corner, 1));

  std::vector<Value> out;
  boost::copy(rng, std::back_inserter(out));

  return out.front();
}

std::vector<std::size_t> sort(Rtree rtree) {
  std::vector<std::size_t> out_indices;
  Value current = find_most_top_left(rtree);
  out_indices.push_back(get_index(current));
  rtree.remove(current);

  // TODO("Rewrite into for(std::optional<Value> current =
  // find_most_top_left(rtree); current.has_value();
  // current=find_follower(current))");
  while (1) {
    auto oCurrent = find_follower(current, rtree);
    if (!oCurrent) {
      break;
    }
    current = *oCurrent;
    out_indices.push_back(get_index(current));
    rtree.remove(current);
  }

  return out_indices;
}

} // namespace

std::vector<std::size_t>
sort_paths_by_endpoints(std::vector<PathEndpoints> endpoints) {
  if (endpoints.empty()) {
    return {};
  }
  if (endpoints.size() == 1) {
    return {0};
  }
  auto rtree = make_rtree(std::move(endpoints));
  return sort(std::move(rtree));
}

