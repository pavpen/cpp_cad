#include "config.h"

#include <iostream>

#include <CGAL/Projection_traits_yz_3.h>

#include <cpp_cad.h>


using cpp_cad::Nef_polyhedron_3;
using cpp_cad::Vector_3;
using cpp_cad::Aff_transformation_3;
using cpp_cad::Polygon_2;
using cpp_cad::Point_2;
using cpp_cad::transform;


// Outputs a torus-like rotational extrusion:
int main()
{
    // Construct a triangle:
    Polygon_2 P1 {
        { 20,    20},
        { 10,     0},
        {  0.1,  10}
    };

    // Extrude the polygon by rotating it around the z axis:
    Nef_polyhedron_3 extrusion = P1.rotate_extrude();

    extrusion.write_to_obj_file("output.obj");

    return 0;
}
