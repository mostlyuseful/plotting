#include "edge.hpp"

Edge Edge::from_line(Line line) {
    auto const ymin = std::min(line.p1.y(), line.p2.y());
    auto const ymax = std::max(line.p1.y(), line.p2.y());
    return Edge{std::move(line), ymin, ymax, line.inv_slope(), {}};
}
