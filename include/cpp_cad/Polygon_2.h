#ifndef _CPP_CAD_POLYGON_2_H
#define _CPP_CAD_POLYGON_2_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include "reference_frame.h"


namespace cpp_cad
{
    class Nef_polyhedron_3;

    class Polygon_2 : public CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>
    {
        public:

        typedef CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>::Traits Traits;

        Polygon_2(const Traits &p_traits=Traits())
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>(p_traits)
        {}

        Polygon_2(const CGAL::Polygon_2<CGAL::Projection_traits_xy_3 <Kernel>> &polygon)
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3 <Kernel>>(polygon)
        {}

        Polygon_2(std::initializer_list<cpp_cad::Point_2> points)
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>(Traits())
        {
            static_cast<Container>(container()).reserve(points.size());

            for (const auto &it : points)
            {
                push_back(it);
            }
        }

        Polygon_2(std::initializer_list<Point_3> points)
            : CGAL::Polygon_2<CGAL::Projection_traits_xy_3<Kernel>>(Traits())
        {
            static_cast<Container>(container()).reserve(points.size());

            for (const auto &it : points)
            {
                insert(vertices_end(), it);
            }
        }

        inline void push_back(const cpp_cad::Point_2 &x)
        {
            insert(vertices_end(), Point_3(x.x(), x.y(), 0));
        }

        Nef_polyhedron_3 linear_extrude(Kernel::FT height);

        Nef_polyhedron_3 rotate_extrude(double angle = 2 * M_PI, int subdivision_c = 16);
    };

}

#endif // _CPP_CAD_POLYGON_2_H