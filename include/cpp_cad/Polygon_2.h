#ifndef _CPP_CAD_POLYGON_2_H
#define _CPP_CAD_POLYGON_2_H

#include <operation_log.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include "reference_frame.h"


namespace cpp_cad
{
    class Nef_polyhedron_3;

    class Polygon_2 : public CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>
    {
        public:

        typedef CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>> CGAL_Polygon_2;
        typedef CGAL_Polygon_2::Traits Traits;

        static Polygon_2 make_square(Kernel::FT x_length, Kernel::FT y_length)
        {
            return Polygon_2 {
                    {       0,        0},
                    {x_length,        0},
                    {x_length, y_length},
                    {       0, y_length}
                };
        }

        static Polygon_2 make_circle(double r, int subdivision_c = 16)
        {
            Polygon_2 res;

            static_cast<Container>(res.container()).reserve(subdivision_c);

            double angle_step = 2 * M_PI / subdivision_c;
            double angle = 0;

            for (int c = 0; c < subdivision_c; ++c, angle += angle_step)
            {
                res.insert(res.vertices_end(), Point_3(
                    r * cos(angle),
                    r * sin(angle),
                    0
                ));
            }

            return res;
        }

        using CGAL_Polygon_2::CGAL_Polygon_2;

        inline Polygon_2(const Traits &p_traits=Traits())
        : CGAL_Polygon_2(p_traits)
        {}

        inline Polygon_2(const CGAL::Polygon_2<CGAL::Projection_traits_xy_3 <Kernel>> &polygon)
        : CGAL_Polygon_2(polygon)
        {}

        // Copy constructor:
        inline Polygon_2(const Polygon_2 &source)
        : CGAL_Polygon_2(source)
        {}

        // Move constructor:
        inline Polygon_2(Polygon_2 &&source)
        {
            // Hack a work-around move constructor:
            Container &d_container = const_cast<Container&>(container());

            d_container = std::move(source.container());
        }

        // Move constructor:
        inline Polygon_2(CGAL_Polygon_2 &&source)
        {
            // Hack a work-around move constructor:
            Container &d_container = const_cast<Container&>(container());

            d_container = std::move(source.container());
        }

        // Move operator:
        inline Polygon_2& operator=(CGAL_Polygon_2 &&source)
        {
            // Hack a work-around move constructor:
            Container &d_container = const_cast<Container&>(container());

            d_container = std::move(source.container());

            return *this;
        }

        Polygon_2(std::initializer_list<cpp_cad::Point_2> points)
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>(Traits())
        {
            reserve(points.size());
            for (const auto &it : points)
            {
                this->push_back(it);
            }
        }

        Polygon_2(std::initializer_list<Point_3> points)
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>(Traits())
        {
            reserve(points.size());

            insert(vertices_end(), points.begin(), points.end());
        }

        inline void reserve(int size)
        {
            // Hack a work-around for reserving polygon points:
            Container &d_container = const_cast<Container&>(container());

            d_container.reserve(size);
        }

        inline void emplace_back(const cpp_cad::Point_3 &&x)
        {
            // Hack a work-around for emplacing polygon points:
            Container &d_container = const_cast<Container&>(container());

            d_container.emplace_back(x);
        }

        inline void push_back(const cpp_cad::Point_2 &x)
        {
            this->emplace_back(Point_3(x.x(), x.y(), 0));
        }

        // Applies an affine transformation to the polygon.
        Polygon_2 &transform(const Aff_transformation_3 &transformation)
        {
            Vertex_const_iterator vit;

            for (vit = vertices_begin(); vit != vertices_end(); ++vit)
            {
                set(vit, vit->transform(transformation));
            }

            return *this;
        }

        Polygon_2 &translate(Kernel::FT x, Kernel::FT y, Kernel::FT z = 0)
        {
            return transform(cpp_cad::Aff_transformation_3::translate(x, y, z));
        }

        Nef_polyhedron_3 linear_extrude(Kernel::FT height);

        Nef_polyhedron_3 rotate_extrude(double angle = 2 * M_PI, int subdivision_c = 16);
    };

}

#endif // _CPP_CAD_POLYGON_2_H