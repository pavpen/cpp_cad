#ifndef _CPP_CAD_POLYHEDRON_3_H
#define _CPP_CAD_POLYHEDRON_3_H

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include "reference_frame.h"
#include "Cube_3_Modifier.h"
#include "Cylinder_3_TessalationModifier.h"
#include "Sphere_3_TessalationModifier.h"


namespace cpp_cad
{
    class Polyhedron_3 : public CGAL::Polyhedron_3<Kernel>
    {
        public:

        static Polyhedron_3 make_cube(Coordinate x_length, Coordinate y_length, Coordinate z_length)
        {
            Polyhedron_3 res;

            res.add_cube(x_length, y_length, z_length);

            return res;
        }

        void add_terahedron(Coordinate circumsphere_r = 1)
        {
            Coordinate t4_edge = 2.0 * circumsphere_r / sqrt(3.0);
            Coordinate t4_face_center_to_edge = circumsphere_r / 3.0;
            Coordinate t4_face_h = circumsphere_r;
            Coordinate t4_h = 2.0 * sqrt(2.0) / 3.0 * circumsphere_r;
            Coordinate sph_center_to_face_center = t4_h - circumsphere_r;

            Polyhedron_3::Halfedge_handle h =
                this->make_tetrahedron(
                    Point_3(
                        -t4_edge / 2.0,
                        t4_face_center_to_edge,
                        -sph_center_to_face_center),
                    Point_3(
                        +t4_edge / 2.0,
                        t4_face_center_to_edge,
                        -sph_center_to_face_center),
                    Point_3(
                        0,
                        -t4_face_h + t4_face_center_to_edge,
                        -sph_center_to_face_center),
                    Point_3(
                        0,
                        0,
                        circumsphere_r)
                );
        }

        void add_cube(Coordinate x_length, Coordinate y_length, Coordinate z_length)
        {
            Cube_3_Modifier<Polyhedron_3::HalfedgeDS>
                modifier(*this, x_length, y_length, z_length);

            this->delegate(modifier);
        }

        void add_cylindrical_tessalation(
            Coordinate base_r = 1, Coordinate top_r = 1, Coordinate height = 1,
            int linear_subdivision_c = 2)
        {
            Cylinder_3_TessalationModifier<Polyhedron_3::HalfedgeDS>
                tessalator(*this, base_r, top_r, height, linear_subdivision_c);

            this->delegate(tessalator);
        }

        void add_spherical_tessalation(Coordinate circumsphere_r = 1, int linear_subdivision_c = 2)
        {
            Sphere_3_TessalationModifier<Polyhedron_3::HalfedgeDS>
                tessalator(*this, circumsphere_r, linear_subdivision_c);

            this->delegate(tessalator);
        }
    };
}


#endif // _CPP_CAD_POLYHEDRON_3_H