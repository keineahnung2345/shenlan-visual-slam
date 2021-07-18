cmake_minimum_required(VERSION 2.8)
project(vo1)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
# g2o https://github.com/RainerKuemmerle/g2o/issues/498
set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#find_package(OpenCV 3 REQUIRED)
find_package(OpenCV REQUIRED)
set(G2O_DIR "/usr/local/include")
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(BA BA.cpp)
target_link_libraries(BA
        g2o_core g2o_stuff g2o_types_sba
        ${OpenCV_LIBS} fmt::fmt)
