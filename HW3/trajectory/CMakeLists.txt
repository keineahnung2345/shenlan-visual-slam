cmake_minimum_required(VERSION 3.0)
project(draw_trajectory)

find_package(Sophus REQUIRED)
#find_package(fmt REQUIRED)
find_package(Pangolin REQUIRED)

#set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-O3")

#message(STATUS "Pangolin_INCLUDE_DIRS dir: ${Pangolin_INCLUDE_DIRS}")
#message(STATUS "Pangolin_LIBRARIES dir: ${Pangolin_LIBRARIES}")

include_directories("/usr/include/eigen3")
#include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory Sophus::Sophus ${Pangolin_LIBRARIES} fmt::fmt)