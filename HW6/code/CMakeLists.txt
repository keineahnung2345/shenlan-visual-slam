project(visual_odometry)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

message(STATUS "Pangolin_INCLUDE_DIRS dir: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin_LIBRARIES dir: ${Pangolin_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method  ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Sophus::Sophus fmt::fmt)