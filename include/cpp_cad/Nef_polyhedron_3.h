#ifndef _CPP_CAD_NEF_POLYHEDRON_3_H
#define _CPP_CAD_NEF_POLYHEDRON_3_H

#include <fstream>

#include <CGAL/IO/print_wavefront.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Aff_transformation_3.h>

#include "Aff_transformation_3.h"
#include "reference_frame.h"
#include "Polyhedron_3/Polyhedron_3.h"


namespace cpp_cad
{

    class Nef_polyhedron_3 : public CGAL::Nef_polyhedron_3<Kernel>
    {
        public:

        inline static Nef_polyhedron_3 make_cube(
            Coordinate x_length, Coordinate y_length, Coordinate z_length)
        {
            Polyhedron_3 p;

            p.add_cube(x_length, y_length, z_length);

            return Nef_polyhedron_3(p);
        }

        inline static Nef_polyhedron_3 make_cylinder(
            Coordinate base_r = 1, Coordinate top_r = 1, Coordinate height = 1,
            int linear_subdivision_c = 2)
        {
            Polyhedron_3 p;

            p.add_cylindrical_tessalation(base_r, top_r, height, linear_subdivision_c);

            return Nef_polyhedron_3(p);
        }

        inline static Nef_polyhedron_3 make_sphere(
            Coordinate circumsphere_r = 1, int linear_subdivision_c = 2)
        {
            Polyhedron_3 p;

            p.add_spherical_tessalation(circumsphere_r, linear_subdivision_c);

            return Nef_polyhedron_3(p);
        }

        using CGAL::Nef_polyhedron_3<Kernel>::Nef_polyhedron_3;

        Nef_polyhedron_3(CGAL::Nef_polyhedron_3<Kernel>::Content space=EMPTY)
            : CGAL::Nef_polyhedron_3<Kernel>(space)
        {}

        Nef_polyhedron_3(CGAL::Nef_polyhedron_3<Kernel> &p)
            : CGAL::Nef_polyhedron_3<Kernel>(p)
        {}

        Nef_polyhedron_3(CGAL::Nef_polyhedron_3<Kernel> p)
            : CGAL::Nef_polyhedron_3<Kernel>(p)
        {}

        Nef_polyhedron_3 &transform(
            cpp_cad::Aff_transformation_3 transformation)
        {
            CGAL::Nef_polyhedron_3<Kernel>::transform(transformation);

            return *this;
        }

        Nef_polyhedron_3 &translate(Kernel::FT x, Kernel::FT y, Kernel::FT z)
        {
            return transform(cpp_cad::Aff_transformation_3::translate(x, y, z));
        }

        Nef_polyhedron_3 &rotate_x(double angle)
        {
            return transform(cpp_cad::Aff_transformation_3::rotate_x(angle));
        }

        Nef_polyhedron_3 &rotate_y(double angle)
        {
            return transform(cpp_cad::Aff_transformation_3::rotate_y(angle));
        }

        Nef_polyhedron_3 &rotate_z(double angle)
        {
            return transform(cpp_cad::Aff_transformation_3::rotate_z(angle));
        }

        void write_to_obj_file(std::string path)
        {
            CGAL::Polyhedron_3<Kernel> p;
            convert_to_polyhedron(p);

            std::ofstream output_f(path);

            CGAL::print_wavefront(output_f, p);
        }
    };

}

#endif // _CPP_CAD_NEF_POLYHEDRON_3_H
