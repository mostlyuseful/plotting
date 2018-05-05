#ifndef POLYFILL_H
#define POLYFILL_H

#include <debugbreak.h>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

// #include <optional>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <range/v3/core.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/action/unique.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/tail.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/unique.hpp>
#include <range/v3/view/zip.hpp>

#include <deque>
#include <limits>
#include <queue>
#include <set>
#include <vector>

using Eigen::Array2d;
using Eigen::Array3d;
using Eigen::ArrayXd;
using namespace ranges;

struct ParametricLine {
    /// Ortsvektor
    Array2d p;
    /// Richtungsvektor
    Array2d u;

    inline Array2d eval(double s) const {
        return p + s*u;
    }

    inline double solve_for_s(Array2d const& query_point) const {
        Eigen::Matrix<double, 2,1> a = u.matrix().eval();
        Eigen::Matrix<double, 2,1> b = query_point-p;
        auto const result = a.colPivHouseholderQr().solve(b).eval();
        return result(0,0);
    }

};

inline Array3d homogenous_form(Array2d const& p1, Array2d const& p2) {
    Eigen::Vector3d const a(p1(0), p1(1), 1.0);
    Eigen::Vector3d const b(p2(0), p2(1), 1.0);
    Eigen::Vector3d const c = a.cross(b);
    return c;
}

inline ParametricLine parametric_form(Array2d const& p1, Array2d const& p2) {
    // Directional vector
    Array2d const u(p2(0)-p1(0),p2(1)-p1(1));
    // Positional vector
    Array2d const p = p1;
    return ParametricLine{p, u};
}

struct Line {
    Array3d homogenous;
    ParametricLine parametric;
    Array2d p1;
    Array2d p2;

    static inline Line from_endpoints(Array2d const& p1, Array2d const& p2) {
        return Line{homogenous_form(p1,p2), parametric_form(p1,p2), p1, p2};
    }
};

struct IntersectionPoint {
    Array2d location;
    bool on_l1;
    bool on_l2;
};

enum class DenormalIntersection {
    nowhere,
    everywhere
};

using Intersection = boost::variant<IntersectionPoint, DenormalIntersection>;

inline Intersection intersect_lines(Line const& l1, Line const& l2) {
    Array3d const ix = l1.homogenous.matrix().cross(l2.homogenous.matrix()).array();
    if(ix.isApproxToConstant(0)) {
        return DenormalIntersection::everywhere;
    } else if(ix.tail<1>().isApproxToConstant(0)) {
        return DenormalIntersection::nowhere;
    }
    // Now check whether point lies on both lines
    Array2d const location(ix(0)/ix(2), ix(1)/ix(2));
    auto const s1 = l1.parametric.solve_for_s(location);
    auto const s2 = l2.parametric.solve_for_s(location);
    bool const on_l1 = (s1>=0) && (s1<=1);
    bool const on_l2 = (s2>=0) && (s2<=1);
    return IntersectionPoint{location, on_l1, on_l2};
}

struct Span {
    double x0;
    double x1;
};

struct HorizontalSlope {};
struct FiniteSlope {
    double value;
};
using Slope = boost::variant<FiniteSlope, HorizontalSlope>;

class holds_finite_slope_visitor
    : public boost::static_visitor<bool>
{
public:

    bool operator()(HorizontalSlope const& s) const
    {
        return false;
    }

    bool operator()(FiniteSlope const& s) const
    {
        return true;
    }

};

struct Edge {
    Line line;
    double ymin;
    double ymax;
    Slope slope;
    boost::optional<Array2d> last_intersection_point;

    inline Edge(Line line, double ymin, double ymax, Slope slope, boost::optional<Array2d> last_intersection_point)
        : line(line)
        , ymin(ymin)
        , ymax(ymax)
        , slope(slope)
        , last_intersection_point(last_intersection_point)
    {}
};

struct Polygon {
    ArrayXd xs;
    ArrayXd ys;

    inline Polygon(ArrayXd x, ArrayXd y) {
        // Is polygon closed?
        // We store coordinates in unclosed fashion, algorithms depend on that!
        bool const xsame = (x.tail(1)(0) == x.head(1)(0));
        bool const ysame = (y.tail(1)(0) == y.head(1)(0));
        bool const closed = xsame && ysame;
        if(closed) {
            x.conservativeResize(x.size()-1);
            y.conservativeResize(y.size()-1);
        }
        this->xs = std::move(x);
        this->ys = std::move(y);
    }
    inline double ymin() const {
        return ys.minCoeff();
    }
    inline double ymax() const {
        return ys.maxCoeff();
    }
    inline std::vector<Line> lines() const {
        auto const n = xs.size();
        std::vector<Line> out;
        for (Eigen::Index i = 0; i<n; ++i) {
            auto const j = (i+1)%n;
            auto const p1 = Array2d(xs(i), ys(i));
            auto const p2 = Array2d(xs(j), ys(j));
            out.push_back(Line::from_endpoints(p1,p2));
        }
        return out;
    }
    inline Polygon reversed() const {
        return {xs.reverse(), ys.reverse()};
    }
};

inline std::vector<Span> get_spans(Polygon const& polygon, double const y) {
    Eigen::Vector3d const scanline(0,1,-y);
    auto const lines = polygon.lines();
    std::vector<double> xx_raw;
    for (auto const& polyline : lines) {
        Array3d const ix = scanline.cross(polyline.homogenous.matrix()).array();
        if(ix.tail<1>().isApproxToConstant(0)) {
            continue;
        }
        auto const ixp = Array2d(ix(0)/ix(2), ix(1)/ix(2));
        auto const s = polyline.parametric.solve_for_s(ixp);
        if ((s>=0) && (s<=1)) {
            xx_raw.push_back(ixp.x());
        }
    }

    std::set<double> xx_set(xx_raw.cbegin(),xx_raw.cend());
    std::vector<double> xx_sorted(xx_set.cbegin(), xx_set.cend());
    std::sort(xx_sorted.begin(), xx_sorted.end());


    std::vector<Span> spans;
    auto rng = view::zip( xx_sorted | view::stride(2),
                          xx_sorted | view::tail | view::stride(2) );

    for(auto pair : rng) {
        spans.push_back(Span{pair.first,pair.second});
    }

    return spans;
}

inline Slope inv_slope(Line const& l) {
    auto const dy = l.p2.y() - l.p1.y();
    auto const dx = l.p2.x() - l.p1.x();
    if (dy == 0) {
        return HorizontalSlope();
    }

    if(dx==0) {
        return FiniteSlope{0};
    } else {
        return FiniteSlope{dx/dy};
    }
}

inline Edge to_edge(Line const& line) {
    auto const ymin = std::min(line.p1.y(), line.p2.y());
    auto const ymax = std::max(line.p1.y(), line.p2.y());
    return Edge{line, ymin, ymax, inv_slope(line), {}};
}

inline std::vector<Edge> sort_edges(Polygon const& p) {
    auto lines = p.lines();
    auto bottom = [](Line const& line) {
        return std::min(line.p1.y(), line.p2.y());
    };
    auto cmp = [&bottom](Line const& a, Line const& b) {
        return bottom(a) < bottom(b);
    };
    std::sort(lines.begin(), lines.end(), cmp);
    return lines | view::transform([](Line const& line) {
        return to_edge(line);
    });
}

struct RasterLine {
    double y;
    std::vector<Span> spans;

    inline RasterLine()
        : y(std::numeric_limits<double>::quiet_NaN())
        , spans()
    {}
    inline RasterLine(double y, std::vector<Span> spans)
        : y(y)
        , spans(std::move(spans))
    {}
};

inline std::vector<RasterLine> raster_polygon(Polygon const& p, double dy = 1) {

    //debug_break();

    if(p.xs.size()!=p.ys.size()) {
        return {};
    }

    if(p.xs.size()<3) {
        return {};
    }

    if(dy<=0) {
        return {};
    }


    std::vector<RasterLine> output;

    std::deque<Edge> remaining_edges = [&p]() {
        auto vec = sort_edges(p);
        std::deque<Edge> q(vec.begin(), vec.end());
        return q;
    }();
    std::vector<Edge> active_edges;

    auto discard_stale_edges = [](std::vector<Edge> const& active_edges, double const y) {
        return active_edges | view::filter([y](Edge const& e) -> bool{
            return (e.ymin < y) && (y <= e.ymax);
        });
    };

    auto fill_active_edges = [](
                                 std::deque<Edge> const& remaining_edges,
                                 std::vector<Edge> const& active_edges,
    double const y) {
        std::deque<Edge> new_remaining_edges(remaining_edges);
        std::vector<Edge> new_active_edges(active_edges);
        while (!new_remaining_edges.empty()) {
            auto const& front = new_remaining_edges.front();
            if((front.ymin<y) && (y<=front.ymax)) {
                new_active_edges.push_back(front);
                new_remaining_edges.pop_front();
            } else {
                break;
            }
        }
        return std::make_pair(new_remaining_edges, new_active_edges);
    };

    auto const ymin = p.ymin();
    auto const ymax = p.ymax();
    for(double y=ymin; y<ymax+dy; y+=dy) {
        active_edges = discard_stale_edges(active_edges, y);
        std::tie(remaining_edges, active_edges) = fill_active_edges(remaining_edges, active_edges, y);
        std::vector<double> intersections;
        for(auto& e : active_edges) {
            if(!e.last_intersection_point.is_initialized()) {
                Eigen::Vector3d const scanline(0,1,-y);
                Array3d const ix = scanline.cross(e.line.homogenous.matrix()).array();
                if(!ix.tail<1>().isApproxToConstant(0)) {
                    auto const ixp = Array2d(ix(0)/ix(2), ix(1)/ix(2));
                    auto const s = e.line.parametric.solve_for_s(ixp);
                    if ((s>=0) && (s<=1)) {
                        intersections.push_back(ixp.x());
                        e.last_intersection_point = ixp;
                    }
                }
            } else {

                if(boost::apply_visitor(holds_finite_slope_visitor(), e.slope)) {
                    //if(std::holds_alternative<FiniteSlope>(e.slope)) {
                    auto const old_ixp = *(e.last_intersection_point);
                    auto const local_dy = y - old_ixp.y();
                    auto const& slope = boost::get<FiniteSlope>(e.slope);
                    auto const new_ixp = Array2d(local_dy*slope.value+old_ixp.x(), y);
                    intersections.push_back(new_ixp.x());
                    e.last_intersection_point = new_ixp;
                } else {
                    debug_break();
                }
            }
        }

        intersections |= action::sort | action::unique;
        std::vector<Span> spans;
        auto rng = view::zip( intersections | view::stride(2),
                              intersections | view::tail | view::stride(2) );
        for(auto pair : rng) {
            spans.push_back(Span{pair.first,pair.second});
        }
        output.emplace_back(y, std::move(spans));
    }

    return output;

}

inline std::vector<RasterLine> raster_polygon(
    std::vector<double> xx,
    std::vector<double> yy,
    double dy = 1) {
    auto&& xs = Eigen::Map<ArrayXd>(xx.data(), static_cast<Eigen::Index>(xx.size()));
    auto&& ys = Eigen::Map<ArrayXd>(yy.data(), static_cast<Eigen::Index>(yy.size()));
    Polygon pgon(xs, ys);
    return raster_polygon(pgon, dy);
}

template <typename TRangePaths>
inline std::vector<Polygon> merge_close_paths(TRangePaths paths) {
    
}

#endif // POLYFILL_H
