cmake_minimum_required(VERSION 3.15)
project(cpp_module)

find_package(Python REQUIRED COMPONENTS Interpreter Development)

add_subdirectory(extern/nanobind)

nanobind_add_module(cpp_module sense.cpp)
nanobind_add_module(cpp_module sense.cpp intersect.c)