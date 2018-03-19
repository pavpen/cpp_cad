# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project( linear_extrude )


cmake_minimum_required(VERSION 2.8.11)


add_subdirectory(linear_extrude)
add_subdirectory(rotate_extrude)
add_subdirectory(solid_difference)
