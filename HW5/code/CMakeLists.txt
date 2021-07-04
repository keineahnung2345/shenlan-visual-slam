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

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt Sophus::Sophus fmt::fmt)

add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA Sophus::Sophus fmt::fmt)

# add_executable(ICP ICP.cpp)
# target_link_libraries(ICP Sophus::Sophus ${Pangolin_LIBRARIES} fmt::fmt)