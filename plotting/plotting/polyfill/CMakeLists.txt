cmake_minimum_required(VERSION 2.6)
project(polyfill)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

add_executable(polyfill-test
    main.cpp
)

target_include_directories(
  polyfill-test PUBLIC 
  ${PROJECT_SOURCE_DIR}/external/range-v3/include
  ${PROJECT_SOURCE_DIR}/external/debugbreak
)
