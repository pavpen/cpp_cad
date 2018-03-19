#ifndef _CPP_CAD_POLYGON_2_CPP
#define _CPP_CAD_POLYGON_2_CPP

#include <CGAL/Polygon_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include "Nef_polyhedron_3.h"
#include "reference_frame.h"

#include "Polygon_2.h"


namespace cpp_cad
{
    Nef_polyhedron_3 Polygon_2::linear_extrude(Kernel::FT height)
    {
        Polyhedron_3 p;

        p.add_linear_extrusion(*this, height);

        return Nef_polyhedron_3(p);
    }

    Nef_polyhedron_3 Polygon_2::rotate_extrude(double angle, int subdivision_c)
    {
        Polyhedron_3 p;

        p.add_rotate_extrusion(*this, angle, subdivision_c);

        return Nef_polyhedron_3(p);
    }
}

#endif // _CPP_CAD_POLYGON_2_CPP