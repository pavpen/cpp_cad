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

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -DOPERATION_LOG_ENABLE" )

add_executable(
    CubicBezierSegment-cusp-and-inflection-calculator-test
    CubicBezierSegment-cusp-and-inflection-calculator-test.cpp)
target_include_directories(CubicBezierSegment-cusp-and-inflection-calculator-test PUBLIC
    "../../../include")
target_link_libraries(CubicBezierSegment-cusp-and-inflection-calculator-test gtest_main)
add_test(NAME CubicBezierSegment-cusp-and-inflection-calculator-test_test
    COMMAND CubicBezierSegment-cusp-and-inflection-calculator-test)
