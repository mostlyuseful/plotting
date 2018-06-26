cmake_minimum_required(VERSION 2.6)
project(polyfill)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
include(cotire)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

add_executable(polyfill-test
    main.cpp
    blow_up.cpp
    candidate.cpp
    dpath_utils.cpp
    edge.cpp
    intersection.cpp
    line.cpp
    mergepaths.cpp
    merger_rtree.cpp
    parametricline.cpp
    path.cpp
    pnpoly_rtree.cpp
    polygon.cpp
    raster.cpp
    ${PROJECT_SOURCE_DIR}/polyfill_wrap_pybind.cxx
    ${PROJECT_SOURCE_DIR}/external/polyclipping/cpp/clipper.cpp
)

target_include_directories(
  polyfill-test PUBLIC 
  ${PROJECT_SOURCE_DIR}/external/range-v3/include
  ${PROJECT_SOURCE_DIR}/external/debugbreak
  ${PROJECT_SOURCE_DIR}/external/polyclipping/cpp
  ${PROJECT_SOURCE_DIR}/external/pybind11/include
  ${PROJECT_SOURCE_DIR}/external/cppitertools
  /usr/include/python3.5m
)

target_link_libraries(
    polyfill-test
    python3.5m
)

cotire(polyfill-test)
