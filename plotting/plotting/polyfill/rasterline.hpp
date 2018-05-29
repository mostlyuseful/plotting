#ifndef RASTERLINE_HPP
#define RASTERLINE_HPP

#include <limits>
#include <vector>

class Span {
public:
    double x0;
    double x1;

    inline Span(double x0=std::numeric_limits<double>::quiet_NaN(), double x1=std::numeric_limits<double>::quiet_NaN()): x0(x0), x1(x1){}
    inline Span(Span const& span) = default;
    inline Span(Span&& span) = default;
};

struct RasterLine {
    double y;
    std::vector<Span> spans;

    inline RasterLine()
        : y(std::numeric_limits<double>::quiet_NaN()), spans() {}
    inline RasterLine(double y, std::vector<Span> spans)
        : y(y), spans(std::move(spans)) {}
};

#endif /* RASTERLINE_HPP */
