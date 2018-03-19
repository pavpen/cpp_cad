#ifndef _CPP_CAD_CYLINDER_3_TESSALATION_MODIFIER_H
#define _CPP_CAD_CYLINDER_3_TESSALATION_MODIFIER_H

#include "../reference_frame.h"
#include "Cylinder_3_TessalationBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a tessalation of a cylinder to the
// polyhedron.
template <class HDS>
class Cylinder_3_TessalationModifier : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT base_r;
    Kernel::FT top_r;
    Kernel::FT height;
    int linear_subdivisions;
    CGAL::Polyhedron_3<Kernel> polyhedron;

public:
    inline Cylinder_3_TessalationModifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        Kernel::FT base_r = 1, Kernel::FT top_r = 1, Kernel::FT height = 1,
        int linear_subdivisions = 2)
    : base_r(base_r),
        top_r(top_r),
        height(height),
        linear_subdivisions(linear_subdivisions),
        polyhedron(polyhedron),
        CGAL::Modifier_base<HDS>()
    {}

    void operator()(HDS& hds)
    {
        Cylinder_3_TessalationBuilder<HDS> builder(
            polyhedron, hds, base_r, top_r, height, linear_subdivisions);

        builder.run();
    }
};

}

#endif // _CPP_CAD_CYLINDER_3_TESSALATION_MODIFIER_H
