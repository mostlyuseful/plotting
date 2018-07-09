#ifndef SORT_PATHS_HPP
#define SORT_PATHS_HPP

#include <tuple>
#include <vector>

using Endpoint = std::tuple<double, double>;
using PathEndpoints = std::tuple<Endpoint, Endpoint>;

std::vector<std::size_t> sort_paths_by_endpoints(std::vector<PathEndpoints> endpoints);

#endif // SORT_PATHS_HPP
