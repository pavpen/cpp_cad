#ifndef _CPP_CAD_LINEAR_EXTRUSION_BUILDER_H
#define _CPP_CAD_LINEAR_EXTRUSION_BUILDER_H


#include <algorithm>
#include <cassert>

#include <operation_log.h>

#include "../Polygon_2.h"
#include "Polyhedron_3_BuilderBase.h"


namespace cpp_cad
{

// A class that extrudes a polygon in the xy plane into a 3D polyhedron.
template <class HDS>
class LinearExtrusionBuilder : public Polyhedron_3_BuilderBase<HDS>
{
protected:
    using Polyhedron_3_BuilderBase<HDS>::builder;
    OPERATION_LOG_CODE(
        using Polyhedron_3_BuilderBase<HDS>::face_count;
    )

private:
    Kernel::FT height;
    const Polygon_2 &polygon;

public:
    inline LinearExtrusionBuilder(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds,
        const Polygon_2 &polygon,
        Kernel::FT height = 1)
    : polygon(polygon),
        height(height),
        Polyhedron_3_BuilderBase<HDS>(polyhedron, hds)
    {}

    void run()
    {
        int base_vertex_count = polygon.size();
        int vertex_count = 2 * base_vertex_count;
        // There are base_vertex_count side faces, and 2 end faces:
        int face_count = 2 + base_vertex_count;

        // Each side face has 4 halfedges (4 * base_vertex_count), each
        // end face has base_vertex_count halfedges (2 * base_vertex_count).
        int halfedge_count = 6 * base_vertex_count;

        builder.begin_surface(vertex_count, face_count, halfedge_count);
        add_tessalation();
        builder.end_surface();
    }

protected:
    using Polyhedron_3_BuilderBase<HDS>::add_face;
    using Polyhedron_3_BuilderBase<HDS>::add_vertex;

private:
    void add_tessalation()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        // Add vertices:
        OPERATION_LOG_MESSAGE("Adding top face vertices.");
        add_end_face_vertices(height);
        OPERATION_LOG_MESSAGE("Adding base face vertices.");
        add_base_vertices();

        add_top_face();
        add_side_faces();
        add_base_face();

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_base_vertices()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        typename Polygon_2::Vertex_const_iterator vit;

        for (vit = polygon.vertices_begin();
            vit != polygon.vertices_end();
            ++vit)
        {
            add_vertex(*vit);
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_end_face_vertices(Kernel::FT z)
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        typename Polygon_2::Vertex_const_iterator vit;

        for (vit = polygon.vertices_begin();
            vit != polygon.vertices_end();
            ++vit)
        {
            add_vertex(vit->x(), vit->y(), vit->z() + z);
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_top_face()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        typename Polygon_2::Vertex_const_iterator vit;

        int vertex_index = 0;

        builder.begin_facet();
        for (vit = polygon.vertices_begin();
            vit != polygon.vertices_end();
            ++vit, ++vertex_index)
        {
            builder.add_vertex_to_facet(vertex_index);
        }
        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_side_faces()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        typename Polygon_2::Vertex_const_iterator vit;

        int top_vertex_index = 0;
        int base_vertex_index = polygon.size();
        int last_top_vertex_index = polygon.size() - 1;
        int last_base_vertex_index = 2 * polygon.size() - 1;

        for (vit = polygon.vertices_begin();
            vit != polygon.vertices_end();
            ++vit)
        {
            int next_base_vertex_index = base_vertex_index + 1;

            if (next_base_vertex_index > last_base_vertex_index)
            {
                next_base_vertex_index -= polygon.size();
            }

            int next_top_vertex_index = top_vertex_index + 1;

            if (next_top_vertex_index > last_top_vertex_index)
            {
                next_top_vertex_index -= polygon.size();
            }

            add_face(
                top_vertex_index,
                base_vertex_index,
                next_base_vertex_index,
                next_top_vertex_index
            );

            base_vertex_index = next_base_vertex_index;
            top_vertex_index = next_top_vertex_index;
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_base_face()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        typename Polygon_2::Vertex_const_iterator vit;

        int vertex_index = 2 * polygon.size() - 1;

        builder.begin_facet();
        for (vit = polygon.vertices_begin();
            vit != polygon.vertices_end();
            ++vit, --vertex_index)
        {
            builder.add_vertex_to_facet(vertex_index);
        }
        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }
};

}

#endif // _CPP_CAD_LINEAR_EXTRUSION_BUILDER_H
