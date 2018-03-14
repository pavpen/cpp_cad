// A "Hello, world" and tutorial sample.
//
// "1 Three Points and One Segment" from
// https://doc.cgal.org/latest/Manual/tutorial_hello_world.html
//
// "1 Output Operator" from
// https://doc.cgal.org/latest/Stream_support/index.html
//
// # To Build:
//
// ## Create a CMakeLists:
//
//   cgal_create_CMakeLists -s executable
//
// * Add `SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )` to
// `CMakeLists.txt`.
//
// ## Create a Makefile:
//
//   mkdir build
//   cd build
//   cmake ..
//
// ## Compile:
//
//   make
//
// # Run:
//
// ./output_sphere

#include "config.h"

#include <cpp_cad.h>


typedef cpp_cad::Nef_polyhedron_3 Nef_polyhedron_3;
typedef cpp_cad::Polyhedron_3 Polyhedron_3;


// Outputs the difference between a cube and a sphere as an OBJ file:
int main()
{
    // A 10 x 10 x 10 unit cube:
    Nef_polyhedron_3 nef_cube = Nef_polyhedron_3::make_cube(10, 10, 10);
    // A sphere with radius 10 units, tessalated with 16 divisions per great
    // circle:
    Nef_polyhedron_3 nef_sphere = Nef_polyhedron_3::make_sphere(10, 16);
    // The sphere subtracted from the cube:
    Nef_polyhedron_3 diff = nef_cube - nef_sphere;

    diff.write_to_obj_file("output.obj");

    return 0;
}
