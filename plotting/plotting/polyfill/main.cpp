#include "simpleapi.hpp"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::ArrayXd;

void check_raster(){
    // std::vector<double> xx{1, 2, 3, 0.3, 3.7, 1};
    // std::vector<double> yy{0, 3, 0, 2, 2, 0};
    std::vector<double> xx{10, 20, 20, 10, 10};
    std::vector<double> yy{100, 100, 90, 90, 100};
    auto lines = raster_polygon(xx, yy, 3);
    for(auto const& line: lines) {
        std::cout << "y = " << line.y << ", spans:" << std::endl;
        for(auto const& span: line.spans) {
            std::cout << "\t" << span.x0 << " .. " << span.x1 << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    // check_raster();

    // std::vector<double> xx{1, 2, 3, 0.3, 3.7, 1};
    // std::vector<double> yy{0, 3, 0, 2, 2, 0};
    std::vector<double> xx{10, 20, 20, 10, 10};
    std::vector<double> yy{100, 100, 90, 90, 100};
    auto paths = raster_merge_polygon(xx, yy, 3, 5);
    for(auto const& p: paths) {
        std::cout << "Path = ";
        for(Eigen::Index i=0; i<p.xs.size();++i) {
            double const x = p.xs(i);
            double const y = p.ys(i);
            std::cout << "(" << x << ", " << y << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
