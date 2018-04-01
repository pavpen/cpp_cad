#ifndef _CPP_CAD_AFF_TRANSFORMATION_3_H
#define _CPP_CAD_AFF_TRANSFORMATION_3_H

#include <CGAL/Aff_transformation_3.h>

#include "reference_frame.h"


namespace cpp_cad
{
    class Aff_transformation_3 : public CGAL::Aff_transformation_3<Kernel>
    {
        public:

        using CGAL_Aff_transformation_3 = CGAL::Aff_transformation_3<Kernel>;
        using CGAL_Aff_transformation_3::CGAL_Aff_transformation_3;

        static Aff_transformation_3 translate(
            Kernel::FT x, Kernel::FT y, Kernel::FT z)
        {
            return Aff_transformation_3(
                CGAL::TRANSLATION, CGAL::Vector_3<Kernel>(x, y, z));
        }

        static Aff_transformation_3 translate(const cpp_cad::Point_3 &vec)
        {
            return Aff_transformation_3(CGAL::TRANSLATION, vec);
        }

        static Aff_transformation_3 scale(Kernel::FT f)
        {
            return Aff_transformation_3(
                f, 0, 0,
                0, f, 0,
                0, 0, f
            );
        }

        static Aff_transformation_3 scale(
            Kernel::FT x_factor, Kernel::FT y_factor, Kernel::FT z_factor)
        {
            return Aff_transformation_3(
                x_factor,        0,        0,
                       0, y_factor,        0,
                       0,        0, z_factor
            );
        }

        static Aff_transformation_3 rotate_x(double angle)
        {
            return Aff_transformation_3(
                1,          0,           0,
                0, cos(angle), -sin(angle),
                0, sin(angle),  cos(angle)
            );
        }

        static Aff_transformation_3 rotate_x(double angle, double r)
        {
            return Aff_transformation_3(
                1,          0,           0,              0,
                0, cos(angle), -sin(angle), r * cos(angle),
                0, sin(angle),  cos(angle), r * sin(angle)
            );
        }

        static Aff_transformation_3 rotate_y(double angle)
        {
            return Aff_transformation_3(
                 cos(angle), 0, sin(angle),
                          0, 1,          0,
                -sin(angle), 0, cos(angle)
            );
        }

        static Aff_transformation_3 rotate_y(double angle, double r)
        {
            return Aff_transformation_3(
                 cos(angle), 0, sin(angle), r * sin(angle),
                          0, 1,          0,              0,
                -sin(angle), 0, cos(angle), r * cos(angle)
            );
        }

        static Aff_transformation_3 rotate_z(double angle)
        {
            return Aff_transformation_3(
                cos(angle), -sin(angle), 0,
                sin(angle),  cos(angle), 0,
                         0,           0, 1
            );
        }

        static Aff_transformation_3 rotate_z(double angle, double r)
        {
            return Aff_transformation_3(
                cos(angle), -sin(angle), 0, r * cos(angle),
                sin(angle),  cos(angle), 0, r * sin(angle),
                         0,           0, 1,              0
            );
        }

        static Aff_transformation_3 rotate_y_rotate_z(double y_angle, double z_angle)
        {
            return Aff_transformation_3(
                cos(y_angle) * cos(z_angle), -sin(z_angle), sin(y_angle) * cos(z_angle),
                cos(y_angle) * sin(z_angle),  cos(z_angle), sin(y_angle) * sin(z_angle),
                              -sin(y_angle),             0,                cos(y_angle)
            );
        }

        static Aff_transformation_3 rotate_y_rotate_z_translate(
            double y_angle, double z_angle, double dx, double dy, double dz)
        {
            return Aff_transformation_3(
                cos(y_angle) * cos(z_angle), -sin(z_angle), sin(y_angle) * cos(z_angle), dx,
                cos(y_angle) * sin(z_angle),  cos(z_angle), sin(y_angle) * sin(z_angle), dy,
                              -sin(y_angle),             0,                cos(y_angle), dz
            );
        }

        static const Aff_transformation_3& swap_yz()
        {
            static Aff_transformation_3 res(
                1, 0, 0,
                0, 0, 1,
                0, 1, 0
            );

            return res;
        }

        static const Aff_transformation_3& swap_xz()
        {
            static Aff_transformation_3 res(
                0, 0, 1,
                0, 1, 0,
                1, 0, 0
            );

            return res;
        }

        static const Aff_transformation_3& swap_xy()
        {
            static Aff_transformation_3 res(
                0, 1, 0,
                1, 0, 0,
                0, 0, 1
            );

            return res;
        }

        Aff_transformation_3(const CGAL::Translation, const cpp_cad::Point_3 p)
        :   Aff_transformation_3(
                1, 0, 0, p.x(),
                0, 1, 0, p.y(),
                0, 0, 1, p.z()
            )
        {}
    };
}

#endif // _CPP_CAD_AFF_TRANSFORMATION_3_H
