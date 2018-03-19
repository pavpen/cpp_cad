#ifndef _CPP_CAD_POLYHEDRON_3_BUILDER_BASE_H
#define _CPP_CAD_POLYHEDRON_3_BUILDER_BASE_H

namespace cpp_cad
{

// Common functionality for Polyhedron_3 builders.
template <class HDS>
class Polyhedron_3_BuilderBase : public CGAL::Modifier_base<HDS>
{
protected:
    OPERATION_LOG_CODE(
        int vertex_count;
        int face_count;
    )
    CGAL::Polyhedron_3<Kernel> polyhedron;
    CGAL::Polyhedron_incremental_builder_3<HDS> builder;

public:
    inline Polyhedron_3_BuilderBase(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds)
    : OPERATION_LOG_CODE(
            vertex_count(0),
            face_count(0),
      )
        polyhedron(polyhedron),
        builder(hds, true),
        CGAL::Modifier_base<HDS>()
    {}

    // Required when deriving from CGAL::Modifier_base<HDS> to make this class
    // not abstract:
    void operator()(HDS& hds)
    {}

protected:
    // Adds a triangular face to the polyhedron.
    //     The vertices must have already been added.
    inline void add_face(int v0_index, int v1_index, int v2_index)
    {
        OPERATION_LOG_ENTER_FUNCTION(v0_index, v1_index, v2_index);

        OPERATION_LOG_MESSAGE_STREAM(<< "Face " << face_count << ": " <<
            v0_index << " " << 
            v1_index << " " <<
            v2_index);

        builder.begin_facet();
        builder.add_vertex_to_facet(v0_index);
        builder.add_vertex_to_facet(v1_index);
        builder.add_vertex_to_facet(v2_index);
        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Adds a quadrilateral face to the polyhedron.
    //     The vertices must have already been added.
    inline void add_face(int v0_index, int v1_index, int v2_index, int v3_index)
    {
        OPERATION_LOG_ENTER_FUNCTION(v0_index, v1_index, v2_index);

        OPERATION_LOG_MESSAGE_STREAM(<< "Face " << face_count << ": " <<
            v0_index << " " << 
            v1_index << " " <<
            v2_index << " " <<
            v3_index);

        builder.begin_facet();
        builder.add_vertex_to_facet(v0_index);
        builder.add_vertex_to_facet(v1_index);
        builder.add_vertex_to_facet(v2_index);
        builder.add_vertex_to_facet(v3_index);
        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(Kernel::FT x, Kernel::FT y, Kernel::FT z)
    {
        OPERATION_LOG_ENTER_FUNCTION(CGAL::to_double(x), CGAL::to_double(y), CGAL::to_double(z));

        Kernel::Point_3 point(x, y, z);

        OPERATION_LOG_MESSAGE_STREAM(<<
            "Vertex " << vertex_count << ": " << point);

        builder.add_vertex(point);

        OPERATION_LOG_CODE(
            vertex_count++;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(const Point_3 &point)
    {
        OPERATION_LOG_ENTER_FUNCTION(point);

        OPERATION_LOG_MESSAGE_STREAM(<<
            "Vertex " << vertex_count << ": " << point);

        builder.add_vertex(point);

        OPERATION_LOG_CODE(
            vertex_count++;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }
};


}

#endif // _CPP_CAD_POLYHEDRON_3_BUILDER_BASE_H
