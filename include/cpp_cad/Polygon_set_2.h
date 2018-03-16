#ifndef _CPP_CAD_POLYGON_SET_2_H
#define _CPP_CAD_POLYGON_SET_2_H


#include <vector>

#include <CGAL/Polygon_set_2.h>

#include "Nef_polyhedron_3.h"
#include "reference_frame.h"


namespace cpp_cad
{
    typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;

    class Polygon_set_2 : public CGAL::Polygon_set_2<Kernel>
    {
        Nef_polyhedron_3 linear_extrude(Kernel::FT height)
        {
            // Convert the polygon set to a vector of Polygon_with_holes_2
            // objects:
            std::vector<Polygon_with_holes_2> profile;

            profile.reserve(number_of_polygons_with_holes());
            polygons_with_holes(std::back_inserter(profile));

            // Transform
        }
    };

}

#endif // _CPP_CAD_POLYGON_SET_2_H