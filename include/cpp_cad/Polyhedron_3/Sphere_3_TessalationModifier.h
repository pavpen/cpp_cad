#ifndef _CPP_CAD_SPHERE_3_TESSALATION_MODIFIER_H
#define _CPP_CAD_SPHERE_3_TESSALATION_MODIFIER_H

#include "../reference_frame.h"
#include "Sphere_3_TessalationBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a tessalation of a sphere to the polyhedron.
template <class HDS>
class Sphere_3_TessalationModifier : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT circumsphere_r;
    int linear_subdivisions;
    CGAL::Polyhedron_3<Kernel> polyhedron;

public:
    inline Sphere_3_TessalationModifier(CGAL::Polyhedron_3<Kernel> &polyhedron, Kernel::FT circumsphere_r = 1, int linear_subdivisions = 2)
    : CGAL::Modifier_base<HDS>(),
        circumsphere_r(circumsphere_r),
        linear_subdivisions(linear_subdivisions),
        polyhedron(polyhedron)
    {}

    void operator()(HDS& hds)
    {
        Sphere_3_TessalationBuilder<HDS> builder(polyhedron, hds, circumsphere_r, linear_subdivisions);

        builder.run();
    }
};

}

#endif // _CPP_CAD_SPHERE_3_TESSALATION_MODIFIER_H
