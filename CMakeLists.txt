cmake_minimum_required(VERSION 3.13)
project(inertia_calc)

set(CMAKE_CXX_STANDARD 14)

add_executable(inertia_calc main.cpp
                            tinyxml2/tinyxml2.cpp)