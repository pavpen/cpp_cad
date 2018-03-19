#ifndef _CPP_CAD_CUBE_3_MODIFIER_H
#define _CPP_CAD_CUBE_3_MODIFIER_H

#include "../reference_frame.h"
#include "Cube_3_Builder.h"

namespace cpp_cad
{

// A polyhedron modifier that adds a cube to the polyhedron.
template <class HDS>
class Cube_3_Modifier : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT x_length;
    Kernel::FT y_length;
    Kernel::FT z_length;
    CGAL::Polyhedron_3<Kernel> polyhedron;

public:
    inline Cube_3_Modifier(
        CGAL::Polyhedron_3<Kernel> &polyhedron,
        Kernel::FT x_length = 1, Kernel::FT y_length = 1, Kernel::FT z_length = 1)
    : x_length(x_length),
        y_length(y_length),
        z_length(z_length),
        polyhedron(polyhedron),
        CGAL::Modifier_base<HDS>()
    {}

    void operator()(HDS& hds)
    {
        Cube_3_Builder<HDS> builder(
            polyhedron, hds, x_length, y_length, z_length);

        builder.run();
    }
};

}

#endif // _CPP_CAD_CUBE_3_MODIFIER_H
