#ifndef SIMPLEAPI_HPP
#define SIMPLEAPI_HPP

#include "blow_up.hpp"
#include "dpath.hpp"
#include "dpath_utils.hpp"
#include "mergepaths.hpp"
#include "mergeclosepaths.hpp"
#include "polygon.hpp"
#include "raster.hpp"
#include "sort_paths.hpp"

#include <iostream>
#include <vector>

inline std::vector<RasterLine>
raster_polygon(std::vector<double> xx, std::vector<double> yy, double dy) {
  auto &&xs = Eigen::Map<Eigen::ArrayXd>(xx.data(),
                                         static_cast<Eigen::Index>(xx.size()));
  auto &&ys = Eigen::Map<Eigen::ArrayXd>(yy.data(),
                                         static_cast<Eigen::Index>(yy.size()));
  Polygon pgon(xs, ys);
  return pgon.raster(dy);
}

/*inline std::vector<Path> raster_merge_polygon(std::vector<double> xx,
                                              std::vector<double> yy, double dy,
                                              double max_merge_distance) {
  auto &&xs = Eigen::Map<Eigen::ArrayXd>(xx.data(),
                                         static_cast<Eigen::Index>(xx.size()));
  auto &&ys = Eigen::Map<Eigen::ArrayXd>(yy.data(),
                                         static_cast<Eigen::Index>(yy.size()));
  Polygon pgon(xs, ys);
  std::vector<RasterLine> lines = pgon.raster(dy);
  std::vector<Path> paths = paths_from_raster_lines(lines);
  std::vector<Path> merged_paths =
      merge_close_paths(pgon, paths, max_merge_distance);
  return merged_paths;
}//*/

inline std::vector<Path>
raster_merge_polygon_eo(ClipperLib::DPaths polygon_paths, double dy,
                        double max_merge_distance) {
  std::cout << "START: raster_merge_polygon_eo" << std::endl;
  ClipperLib::DPaths raster_dpaths = raster_polygon_eo(polygon_paths, dy);
  std::cout << "AFTER: raster_polygon_eo" << std::endl;
  std::vector<Path> raster_paths = dpaths_to_paths(raster_dpaths);
  std::cout << "AFTER: dpaths_to_paths" << std::endl;
  auto merged_paths =
      merge_close_paths_eo(polygon_paths, raster_paths, max_merge_distance);
  std::cout << "AFTER: merge_close_paths_eo" << std::endl;
  return merged_paths;
}

inline ClipperLib::DPaths simplify_polygon(ClipperLib::DPaths polygon_paths,
                                           double distance) {
  double const blowUpFactor = 1000;
  auto const blown_up_distance = distance * blowUpFactor;
  ClipperLib::Paths blown_up_input_paths = blow_up(polygon_paths, blowUpFactor);
  ClipperLib::Paths blown_up_output_paths;
  ClipperLib::CleanPolygons(blown_up_input_paths, blown_up_output_paths, blown_up_distance);
  ClipperLib::DPaths deflated_output_paths = blow_up(blown_up_output_paths, 1.0/blowUpFactor);
  return deflated_output_paths;
}

inline std::vector<std::size_t> sort_paths(std::vector<PathEndpoints> endpoints) {
  return sort_paths_by_endpoints(std::move(endpoints));
}



#endif /* SIMPLEAPI_HPP */
