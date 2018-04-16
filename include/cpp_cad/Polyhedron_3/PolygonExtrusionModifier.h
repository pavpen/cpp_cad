#ifndef _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
#define _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H

#include "../reference_frame.h"
#include "PolygonExtrusionBuilder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a polygon extrusion to the polyhedron.
template <class HDS, class PolygonInputIterator>
class PolygonExtrusionModifier : public CGAL::Modifier_base<HDS>
{
private:
    bool closed;
    PolygonInputIterator &track_start;
    const PolygonInputIterator &track_end;
    CGAL::Polyhedron_3<Kernel> polyhedron;

public:
    inline PolygonExtrusionModifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        PolygonInputIterator &track_start,
        const PolygonInputIterator &track_end, bool closed = false)
    : CGAL::Modifier_base<HDS>(),
        closed(closed),
        track_start(track_start),
        track_end(track_end),
        polyhedron(polyhedron)
    {}

    void operator()(HDS& hds)
    {
        PolygonExtrusionBuilder<HDS, PolygonInputIterator> builder(
            polyhedron, hds, track_start, track_end, closed);

        builder.run();
    }
};

}

#endif // _CPP_CAD_POLYGON_EXTRUSION_MODIFIER_H
