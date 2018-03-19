#ifndef _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
#define _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H

#include "../reference_frame.h"
#include "PolygonExtrusionBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a cube to the polyhedron.
template <class HDS, class TransformInputIterator>
class PolygonExtrusionModifier : public CGAL::Modifier_base<HDS>
{
private:
    bool closed;
    const Polygon_2 &polygon;
    TransformInputIterator &trajectory_start;
    const TransformInputIterator &trajectory_end;
    CGAL::Polyhedron_3<Kernel> polyhedron;
    int vertex_count;

public:
    inline PolygonExtrusionModifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        const Polygon_2 &polygon, TransformInputIterator &trajectory_start,
        const TransformInputIterator &trajectory_end, bool closed = false)
    : polygon(polygon),
        trajectory_start(trajectory_start),
        trajectory_end(trajectory_end),
        closed(closed),
        polyhedron(polyhedron),
        vertex_count(0),
        CGAL::Modifier_base<HDS>()
    {}

    void operator()(HDS& hds)
    {
        PolygonExtrusionBuilder<HDS, TransformInputIterator> builder(
            polyhedron, hds, polygon, trajectory_start, trajectory_end, closed);

        builder.run();
    }
};

}

#endif // _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
