cmake_minimum_required( VERSION 2.8 )
project( coordinateTransformation )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable(coordinate_transformation coordinate_transformation.cpp)
