#ifndef _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
#define _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H

#include "../reference_frame.h"
#include "PolygonExtrusionBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a cube to the polyhedron.
template <class HDS>
class PolygonExtrusionModifier : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT height;
    const Polygon_2 &polygon;
    CGAL::Polyhedron_3<Kernel> polyhedron;
    int vertex_count;

public:
    inline PolygonExtrusionModifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        const Polygon_2 &polygon, Kernel::FT height = 1)
    : polygon(polygon),
        height(height),
        polyhedron(polyhedron),
        vertex_count(0),
        CGAL::Modifier_base<HDS>()
    {}

    void operator()(HDS& hds)
    {
        PolygonExtrusionBuilder<HDS> builder(
            polyhedron, hds, polygon, height);

        builder.run();
    }
};

}

#endif // _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
