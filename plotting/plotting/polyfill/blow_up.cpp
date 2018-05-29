#include "blow_up.hpp"

ClipperLib::Path blow_up(ClipperLib::DPath const &input, double const factor) {
    using ClipperLib::cInt;
    ClipperLib::Path output;
    output.reserve(input.size());
    for (auto const &p : input) {
        auto const x = static_cast<cInt>(std::get<0>(p) * factor);
        auto const y = static_cast<cInt>(std::get<1>(p) * factor);
        output.emplace_back(x, y);
    }
    return output;
}

ClipperLib::Paths blow_up(ClipperLib::DPaths const &input,
                          double const factor) {
    ClipperLib::Paths output;
    output.reserve(input.size());
    for (auto const &p : input) {
        output.emplace_back(blow_up(p, factor));
    }
    return output;
}
