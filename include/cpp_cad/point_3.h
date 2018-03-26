#ifndef _CPP_CAD_POINT_3_H
#define _CPP_CAD_POINT_3_H

#include "Aff_transformation_3.h"
#include "reference_frame.h"


namespace cpp_cad
{

// Returns the angle between the vector to a given point and the z axis.
double angle_from_z(const Point_3 &vec)
{
    double x = CGAL::to_double(vec.x());
    double y = CGAL::to_double(vec.y());
    double z = CGAL::to_double(vec.z());
    double norm = sqrt(x * x + y * y + z * z);

    return acos(z / norm);
}

}

#endif // _CPP_CAD_POINT_3_H
