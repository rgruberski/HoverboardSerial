cmake_minimum_required(VERSION 3.25)
project(HoverboardSerial)

set(CMAKE_CXX_STANDARD 17)

set(BOOST_ROOT /usr/lib)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREAD ON)

find_package(Boost REQUIRED COMPONENTS system thread regex)

add_executable(HoverboardSerial main.cpp)
target_link_libraries(HoverboardSerial ${Boost_LIBRARIES})
