#ifndef _CPP_CAD_LINEAR_EXTRUSION_MODIFIER_H
#define _CPP_CAD_LINEAR_EXTRUSION_MODIFIER_H

#include "../reference_frame.h"
#include "LinearExtrusionBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a polygon extrusion along the z axis to the
// polyhedron.
template <class HDS>
class LinearExtrusionModifier : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT height;
    const Polygon_2 &polygon;
    CGAL::Polyhedron_3<Kernel> polyhedron;

public:
    inline LinearExtrusionModifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        const Polygon_2 &polygon, Kernel::FT height = 1)
    : polygon(polygon),
        height(height),
        polyhedron(polyhedron),
        CGAL::Modifier_base<HDS>()
    {}

    void operator()(HDS& hds)
    {
        LinearExtrusionBuilder<HDS> builder(
            polyhedron, hds, polygon, height);

        builder.run();
    }
};

}

#endif // _CPP_CAD_LINEAR_EXTRUSION_MODIFIER_H
