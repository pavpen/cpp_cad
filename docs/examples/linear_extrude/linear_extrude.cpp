#include <iostream>

#include <CGAL/Projection_traits_yz_3.h>

#include <cpp_cad.h>


using cpp_cad::Nef_polyhedron_3;
using cpp_cad::Vector_3;
using cpp_cad::Aff_transformation_3;
using cpp_cad::Polygon_2;
using cpp_cad::Point_2;
using cpp_cad::transform;


// Outputs the difference between a cube and a sphere as an OBJ file:
int main()
{
    // Construct a triangle:
    Polygon_2 P1;
    P1.push_back(Point_2(-1,  1));
    P1.push_back(Point_2( 0, -1));
    P1.push_back(Point_2( 1,  1));

    // Extrude the triangle 10 units in the z direction:
    Nef_polyhedron_3 extrusion1 = P1.linear_extrude(10);

    // Construct the same triangle with the opposite order of vertices:
    Polygon_2 P2 {
        { 1,  1},
        { 0, -1},
        {-1,  1}
    };
    // Translate the triangle 15 units in the z direction:
    P2 = transform(
        Aff_transformation_3(CGAL::TRANSLATION, Vector_3(0, 0, 15)),
        P2);

    // Extrude the triangle 5 units in the z direction:
    Nef_polyhedron_3 extrusion2 = P2.linear_extrude(5);

    // Construct a union of the two extrusions:
    Nef_polyhedron_3 extrusions = extrusion1 + extrusion2;

    extrusions.write_to_obj_file("output.obj");

    return 0;
}
