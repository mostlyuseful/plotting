#include "polyfill.hpp"

#include <eigen3/Eigen/Dense>
#include <vector>

using Eigen::ArrayXd;

int main(int argc, char* argv[]) {
    ArrayXd xs(6); xs << 1, 2, 3, 0.3, 3.7, 1;
    ArrayXd ys(6); ys << 0, 3, 0, 2, 2, 0;
    Polygon polygon(xs, ys);
    auto raster_lines = raster_polygon(polygon, 0.25);
    return 0;
}
