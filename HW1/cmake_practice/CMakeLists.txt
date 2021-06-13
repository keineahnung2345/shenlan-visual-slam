project(cmake_practice)

cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

include_directories(include)
add_library(hello src/hello.cpp)

add_executable(sayHello src/useHello.cpp)
target_link_libraries(sayHello hello)

install(TARGETS hello DESTINATION lib)
install(TARGETS sayHello DESTINATION bin)
