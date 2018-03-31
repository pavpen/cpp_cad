cmake_minimum_required(VERSION 2.8.11)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS core )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()

endif()

# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

add_executable(
    CubicBezierSegment-calculate-cusp-test
    CubicBezierSegment-calculate-cusp-test.cpp)
target_include_directories(CubicBezierSegment-calculate-cusp-test PUBLIC
    "../../../include")
target_link_libraries(CubicBezierSegment-calculate-cusp-test gtest_main)
add_test(NAME CubicBezierSegment-calculate-cusp-test_test
    COMMAND CubicBezierSegment-calculate-cusp-test)
