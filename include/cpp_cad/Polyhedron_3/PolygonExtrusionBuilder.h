#ifndef _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H
#define _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H


#include <algorithm>
#include <cassert>

#include <operation_log.h>

#include "../Polygon_2.h"



namespace cpp_cad
{

// A class that extrudes a polygon in the xy plane into a 3D polyhedron.
template <class HDS>
class PolygonExtrusionBuilder : public CGAL::Modifier_base<HDS>
{
private:
    Kernel::FT height;
    const Polygon_2 &polygon;
    int vertex_count;
    CGAL::Polyhedron_3<Kernel> polyhedron;
    CGAL::Polyhedron_incremental_builder_3<HDS> builder;

public:
    inline PolygonExtrusionBuilder(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds,
        const Polygon_2 &polygon,
        Kernel::FT height = 1)
    : polygon(polygon),
        height(height),
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
        int base_vertex_count = polygon.size();
        int vertex_count = 2 * base_vertex_count;
        int face_count = 2 + base_vertex_count;

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

            builder.begin_facet();
            builder.add_vertex_to_facet(top_vertex_index);
            builder.add_vertex_to_facet(base_vertex_index);
            builder.add_vertex_to_facet(next_base_vertex_index);
            builder.add_vertex_to_facet(next_top_vertex_index);
            builder.end_facet();

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

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(const Point_3 &point)
    {
        OPERATION_LOG_ENTER_FUNCTION(std::to_string(point));

        OPERATION_LOG_MESSAGE_STREAM(<<
            "Vertex " << vertex_count << ": " << point);

        builder.add_vertex(point);
        vertex_count++;

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

#endif // _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H
