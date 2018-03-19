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
    Polygon_2 triangle {
        { 20,    20},
        { 10,     0},
        {  0.1,  10}
    };

    // Construct a 10 unit x 10 unit square:
    Polygon_2 square = Polygon_2::make_square(10, 10);

    square.translate(15, 0);

    // Construct a circle with radius 5 units:
    Polygon_2 circle = Polygon_2::make_circle(5);

    circle.translate(20, 0);

    // Extrude the polygon by rotating it full circle around the z axis:
    Nef_polyhedron_3 scene;

    scene += triangle.rotate_extrude();

    // Extrude the square by rotating it half a circle around the z axis:
    scene += square.rotate_extrude(M_PI).translate(0, 0, 30);

    // Extrude the circle by rotating it a quarter circle around the z axis:
    scene += circle.rotate_extrude(M_PI_2)
        .translate(0, 0, 35).rotate_z(M_PI * 3 / 2);

    scene.write_to_obj_file("output.obj");

    return 0;
}
