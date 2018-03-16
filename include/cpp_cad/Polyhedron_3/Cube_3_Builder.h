#ifndef _CPP_CAD_CUBE_3_BUILDER_H
#define _CPP_CAD_CUBE_3_BUILDER_H


#include <algorithm>
#include <cassert>
#include <operation_log.h>

#include "Cylinder_3_operation_logging.h"


namespace cpp_cad
{

// A class that uses a polyhedron incremental builer to build the faces of a
// cube.
template <class HDS>
class Cube_3_Builder : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT x_length;
    Kernel::FT y_length;
    Kernel::FT z_length;
    int vertex_count;
    CGAL::Polyhedron_3<Kernel> polyhedron;
    CGAL::Polyhedron_incremental_builder_3<HDS> builder;

public:
    inline Cube_3_Builder(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds,
        Kernel::FT x_length = 1,
        Kernel::FT y_length = 1,
        Kernel::FT z_length = 1)
    : x_length(x_length),
        y_length(y_length),
        z_length(z_length),
        vertex_count(0),
        polyhedron(polyhedron),
        CGAL::Modifier_base<HDS>(),
        builder(hds, true)
    {}

    // Required when deriving from CGAL::Modifier_base<HDS> to make this class
    // not abstract:
    void operator()(HDS& hds)
    {}

    void run()
    {
        int vertex_count = 8;
        int face_count = 6;

        // Each face has 4 halfedges.
        int halfedge_count = 4 * face_count;

        builder.begin_surface(vertex_count, face_count, halfedge_count);
        add_tessalation();
        builder.end_surface();
    }

private:
    void add_tessalation()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        // Add vertices:
        OPERATION_LOG_MESSAGE("Adding top square vertices.");
        add_end_square_vertices(z_length);
        OPERATION_LOG_MESSAGE("Adding base square vertices.");
        add_end_square_vertices(0);

        add_faces();

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_end_square_vertices(Kernel::FT z)
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        add_vertex(       0,        0, z);
        add_vertex(x_length,        0, z);
        add_vertex(x_length, y_length, z);
        add_vertex(       0, y_length, z);

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_faces()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        // Vertices 0 to 3 are of the top square,
        // vertices 4 to 7 are of the bottom square.
        
        // Top:
        add_face(0, 1, 2, 3);
        // Front:
        add_face(0, 4, 5, 1);
        // Right:
        add_face(1, 5, 6, 2);
        // Back:
        add_face(2, 6, 7, 3);
        // Left:
        add_face(3, 7, 4, 0);
        // Bottom:
        add_face(7, 6, 5, 4);

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Adds a square face to the polyhedron.
    //     The vertices must have already been added.
    inline void add_face(int v0_index, int v1_index, int v2_index, int v3_index)
    {
        OPERATION_LOG_ENTER_FUNCTION(v0_index, v1_index, v2_index);

        builder.begin_facet();
        builder.add_vertex_to_facet(v0_index);
        builder.add_vertex_to_facet(v1_index);
        builder.add_vertex_to_facet(v2_index);
        builder.add_vertex_to_facet(v3_index);
        builder.end_facet();

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(Kernel::FT x, Kernel::FT y, Kernel::FT z)
    {
        OPERATION_LOG_ENTER_FUNCTION(CGAL::to_double(x), CGAL::to_double(y), CGAL::to_double(z));

        Kernel::Point_3 point(x, y, z);

        OPERATION_LOG_MESSAGE_STREAM(<<
            "Vertex " << vertex_count << ": " << point);

        builder.add_vertex(point);
        vertex_count++;

        OPERATION_LOG_LEAVE_FUNCTION();
    }
};

}

#endif // _CPP_CAD_CUBE_3_BUILDER_H
