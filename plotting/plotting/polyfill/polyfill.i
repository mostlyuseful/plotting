%module polyfill
%{
#include "polyfill.hpp"
%}

%include "std_vector.i"

namespace std {
   %template(DoubleVector) vector<double>;
   %template(SpanVector) vector<Span>;
   %template(RasterLineVector) vector<RasterLine>;
};

struct Span{
    double x0;
    double x1;
};

struct RasterLine {
    double y;
    std::vector<Span> spans;
    RasterLine();
    RasterLine(double y, std::vector<Span> spans);
};

std::vector<RasterLine> raster_polygon(std::vector<double> const& xx,
                                       std::vector<double> const& yy,
                                       double dy = 1);