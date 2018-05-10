#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "edge.hpp"
#include "line.hpp"

#include <debugbreak.h>

#include <eigen3/Eigen/Dense>

#include <range/v3/core.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/action/unique.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/tail.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <deque>
#include <map>
#include <set>

struct Span {
    double x0;
    double x1;
};

struct RasterLine {
    double y;
    std::vector<Span> spans;

    inline RasterLine()
        : y(std::numeric_limits<double>::quiet_NaN()), spans() {}
    inline RasterLine(double y, std::vector<Span> spans)
        : y(y), spans(std::move(spans)) {}
};

struct Polygon {
    Eigen::ArrayXd xs;
    Eigen::ArrayXd ys;

    inline Polygon(Eigen::ArrayXd x, Eigen::ArrayXd y) {
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
            auto const p1 = Eigen::Array2d(xs(i), ys(i));
            auto const p2 = Eigen::Array2d(xs(j), ys(j));
            out.push_back(Line::from_endpoints(p1,p2));
        }
        return out;
    }
    inline Polygon reversed() const {
        return {xs.reverse(), ys.reverse()};
    }

    inline std::vector<Edge> sort_edges() const {
        using namespace ranges;
        auto lines = this->lines();
        auto bottom = [](Line const &line) {
            return std::min(line.p1.y(), line.p2.y());
        };
        auto cmp = [&bottom](Line const &a, Line const &b) {
            return bottom(a) < bottom(b);
        };
        std::sort(lines.begin(), lines.end(), cmp);
        return lines |
               view::transform([](Line const &line) { return Edge::from_line(line); });
    }

    inline std::vector<Span> get_spans(double const y) const {
        using namespace ranges;
        Eigen::Vector3d const scanline(0, 1, -y);
        auto const lines = this->lines();
        std::vector<double> xx_raw;
        for (auto const &polyline : lines) {
            Eigen::Array3d const ix =
                scanline.cross(polyline.homogenous.matrix()).array();
            if (ix.tail<1>().isApproxToConstant(0)) {
                continue;
            }
            auto const ixp = Eigen::Array2d(ix(0) / ix(2), ix(1) / ix(2));
            auto const s = polyline.parametric.solve_for_s(ixp);
            if ((s >= 0) && (s <= 1)) {
                xx_raw.push_back(ixp.x());
            }
        }

        std::set<double> xx_set(xx_raw.cbegin(), xx_raw.cend());
        std::vector<double> xx_sorted(xx_set.cbegin(), xx_set.cend());
        std::sort(xx_sorted.begin(), xx_sorted.end());

        std::vector<Span> spans;
        auto rng = view::zip(xx_sorted | view::stride(2),
                             xx_sorted | view::tail | view::stride(2));

        for (auto pair : rng) {
            spans.push_back(Span{pair.first, pair.second});
        }

        return spans;
    }

    inline std::vector<RasterLine> raster(double dy = 1) const {
        using namespace ranges;

        if (xs.size() != ys.size()) {
            return {};
        }

        if (xs.size() < 3) {
            return {};
        }

        if (dy <= 0) {
            return {};
        }

        std::vector<RasterLine> output;

        std::deque<Edge> remaining_edges = [this]() {
            auto vec = this->sort_edges();
            std::deque<Edge> q(vec.begin(), vec.end());
            return q;
        }();
        std::vector<Edge> active_edges;

        auto discard_stale_edges = [](std::vector<Edge> const &active_edges,
                                      double const y) {
            return active_edges | view::filter([y](Edge const &e) -> bool {
                       return (e.ymin < y) && (y <= e.ymax);
                   });
        };

        auto fill_active_edges = [](std::deque<Edge> const &remaining_edges,
                                    std::vector<Edge> const &active_edges,
                                    double const y) {
            std::deque<Edge> new_remaining_edges(remaining_edges);
            std::vector<Edge> new_active_edges(active_edges);
            while (!new_remaining_edges.empty()) {
                auto const &front = new_remaining_edges.front();
                if ((front.ymin < y) && (y <= front.ymax)) {
                    new_active_edges.push_back(front);
                    new_remaining_edges.pop_front();
                } else {
                    break;
                }
            }
            return std::make_pair(new_remaining_edges, new_active_edges);
        };

        auto const ymin = this->ymin();
        auto const ymax = this->ymax();
        for (double y = ymin; y < ymax + dy; y += dy) {
            active_edges = discard_stale_edges(active_edges, y);
            std::tie(remaining_edges, active_edges) =
                fill_active_edges(remaining_edges, active_edges, y);
            std::vector<double> intersections;
            for (auto &e : active_edges) {
                if (!e.last_intersection_point.is_initialized()) {
                    Eigen::Vector3d const scanline(0, 1, -y);
                    Eigen::Array3d const ix =
                        scanline.cross(e.line.homogenous.matrix()).array();
                    if (!ix.tail<1>().isApproxToConstant(0)) {
                        auto const ixp =
                            Eigen::Array2d(ix(0) / ix(2), ix(1) / ix(2));
                        auto const s = e.line.parametric.solve_for_s(ixp);
                        if ((s >= 0) && (s <= 1)) {
                            intersections.push_back(ixp.x());
                            e.last_intersection_point = ixp;
                        }
                    }
                } else {

                    if (boost::apply_visitor(holds_finite_slope_visitor(),
                                             e.slope)) {
                        // if(std::holds_alternative<FiniteSlope>(e.slope)) {
                        auto const old_ixp = *(e.last_intersection_point);
                        auto const local_dy = y - old_ixp.y();
                        auto const &slope = boost::get<FiniteSlope>(e.slope);
                        auto const new_ixp = Eigen::Array2d(
                            local_dy * slope.value + old_ixp.x(), y);
                        intersections.push_back(new_ixp.x());
                        e.last_intersection_point = new_ixp;
                    } else {
                        debug_break();
                    }
                }
            }

            intersections |= action::sort | action::unique;
            std::vector<Span> spans;
            auto rng = view::zip(intersections | view::stride(2),
                                 intersections | view::tail | view::stride(2));
            for (auto pair : rng) {
                spans.push_back(Span{pair.first, pair.second});
            }
            output.emplace_back(y, std::move(spans));
        }

        return output;
    }
};

inline std::vector<RasterLine>
raster_polygon(std::vector<double> xx, std::vector<double> yy, double dy = 1) {
    auto &&xs =
        Eigen::Map<Eigen::ArrayXd>(xx.data(), static_cast<Eigen::Index>(xx.size()));
    auto &&ys = Eigen::Map<Eigen::ArrayXd>(
        yy.data(), static_cast<Eigen::Index>(yy.size()));
    Polygon pgon(xs, ys);
    return pgon.raster(dy);
}

#endif