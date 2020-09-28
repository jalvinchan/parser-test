cmake_minimum_required(VERSION 2.6)
project(parser-test)

find_package(Eigen3 REQUIRED)
add_subdirectory(src/yaml-cpp)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(main src/main.cpp)

target_link_libraries(main
    yaml-cpp
)