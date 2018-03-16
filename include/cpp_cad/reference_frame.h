#ifndef _CPP_CAD_REFERENCE_FRAME_H
#define _CPP_CAD_REFERENCE_FRAME_H

#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_S2.h>
#include <CGAL/Simple_cartesian.h>


namespace cpp_cad
{

typedef double Coordinate;
// typedef CGAL::Simple_cartesian<Coordinate> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Nef_polyhedron_S2<Kernel>                   Nef_polyhedron_S2;
typedef Kernel::Segment_3                                 Segment_3;
typedef Kernel::Sphere_3                                  Sphere_3;
typedef Kernel::Vector_3                                  Vector_3;
typedef Kernel::Point_3                                   Point_3;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Aff_transformation_3<Kernel>                Aff_transformation_3;

}

#endif // _CPP_CAD_REFERENCE_FRAME_H
