#ifndef _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H
#define _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H


#include <algorithm>
#include <cassert>

#include <operation_log.h>

#include "../Polygon_2.h"
#include "../reference_frame.h"
#include "Polyhedron_3_BuilderBase.h"

#include "PolygonExtrusion_operation_logging.h"



namespace cpp_cad
{

// A class that extrudes a polygon in the xy plane into a 3D polyhedron.
template <class HDS, class TransformInputIterator>
class PolygonExtrusionBuilder : public Polyhedron_3_BuilderBase<HDS>
{
protected:
    using Polyhedron_3_BuilderBase<HDS>::builder;

private:
    bool closed;
    const Polygon_2 &polygon;
    TransformInputIterator &trajectory_start;
    const TransformInputIterator &trajectory_end;
    int polygon_vertex_count;
    int prev_slice_vertex_index;
    int slice_vertex_index;

public:
    inline PolygonExtrusionBuilder(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds,
        const Polygon_2 &polygon,
        TransformInputIterator &trajectory_start,
        const TransformInputIterator &trajectory_end,
        bool closed = false)
    : polygon(polygon),
        trajectory_start(trajectory_start),
        trajectory_end(trajectory_end),
        closed(closed),
        Polyhedron_3_BuilderBase<HDS>(polyhedron, hds)
    {}

    void run()
    {
        polygon_vertex_count = polygon.size();
        int slice_count = trajectory_start.steps_left();
        int vertex_count = slice_count * polygon_vertex_count;
        int side_face_count;
        int end_face_count;
        if (closed)
        {
            side_face_count = (slice_count - 1) * polygon_vertex_count;
            end_face_count = 2;
        }
        else
        {
            side_face_count = slice_count * polygon_vertex_count;
            end_face_count = 0;
        }
        int face_count = side_face_count + end_face_count;

        // Each side face has 4 halfedges, each end face has
        // polygon_vertex_count halfedges.
        int halfedge_count =
                4 * side_face_count + polygon_vertex_count * end_face_count;

        builder.begin_surface(vertex_count, face_count, halfedge_count);
        add_tessalation();
        builder.end_surface();
    }

private:
    void add_tessalation()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        TransformInputIterator &it = trajectory_start;

        OPERATION_LOG_DUMP_VARS(closed, polygon_vertex_count);

        // Add vertices:
        OPERATION_LOG_MESSAGE("Adding first slice vertices.");
        add_slice_vertices(transform(*it, polygon));

        prev_slice_vertex_index = 0;

        if (!closed)
        {
            slice_vertex_index = 0;
            add_start_face();
        }

        slice_vertex_index = polygon_vertex_count;
        for (++it; it != trajectory_end; ++it)
        {
            add_slice_vertices(transform(*it, polygon));
            add_side_faces();
        }

        if (closed)
        {
            add_closing_side_faces();
        }
        else
        {
            add_end_face();
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_slice_vertices(const Polygon_2 &slice)
    {
        OPERATION_LOG_ENTER_FUNCTION(slice);

        typename Polygon_2::Vertex_const_iterator vit;

        for (vit = slice.vertices_begin(); vit != slice.vertices_end(); ++vit)
        {
            add_vertex(*vit);
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_start_face()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        int last_vertex_index = slice_vertex_index;

        builder.begin_facet();

        OPERATION_LOG_MESSAGE_STREAM_OPEN(vertex_msg);
        OPERATION_LOG_MESSAGE_STREAM_WRITE(vertex_msg, << "Face " << face_count << ":");

        for (int vertex_index = slice_vertex_index + polygon_vertex_count - 1;
            vertex_index >= last_vertex_index;
            --vertex_index)
        {
            OPERATION_LOG_MESSAGE_STREAM_WRITE(vertex_msg, << " " << vertex_index);
            builder.add_vertex_to_facet(vertex_index);
        }

        OPERATION_LOG_MESSAGE_STREAM_CLOSE(vertex_msg);

        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )
        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_end_face()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        builder.begin_facet();

        int last_vertex_index = slice_vertex_index - 1;

        OPERATION_LOG_MESSAGE_STREAM_OPEN(vertex_msg);
        OPERATION_LOG_MESSAGE_STREAM_WRITE(vertex_msg, << "Face " << face_count << ":");

        for (int vertex_index = prev_slice_vertex_index;
            vertex_index <= last_vertex_index;
            ++vertex_index)
        {
            OPERATION_LOG_MESSAGE_STREAM_WRITE(vertex_msg, << " " << vertex_index);
            builder.add_vertex_to_facet(vertex_index);
        }

        OPERATION_LOG_MESSAGE_STREAM_CLOSE(vertex_msg);

        builder.end_facet();

        OPERATION_LOG_CODE(
            ++face_count;
        )
        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_side_faces()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        OPERATION_LOG_DUMP_VARS(slice_vertex_index, prev_slice_vertex_index, polygon_vertex_count);

        int slice_last_vertex_index = slice_vertex_index + polygon_vertex_count - 1;

        while (slice_vertex_index < slice_last_vertex_index)
        {
            int slice_next_vertex_index = slice_vertex_index + 1;
            int prev_slice_next_vertex_index = prev_slice_vertex_index + 1;

            this->add_face(
                slice_vertex_index,
                prev_slice_vertex_index,
                prev_slice_next_vertex_index,
                slice_next_vertex_index
            );

            slice_vertex_index = slice_next_vertex_index;
            prev_slice_vertex_index = prev_slice_next_vertex_index;
        }

        // Add the last face:
        OPERATION_LOG_MESSAGE("Adding last face.");

        this->add_face(
            slice_vertex_index,
            prev_slice_vertex_index,
            prev_slice_vertex_index + 1 - polygon_vertex_count,
            slice_vertex_index + 1 - polygon_vertex_count
        );

        ++slice_vertex_index;
        ++prev_slice_vertex_index;

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_closing_side_faces()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        slice_vertex_index = 0;
        add_side_faces();

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(const Point_3 &point)
    {
        OPERATION_LOG_ENTER_FUNCTION(point);

        Polyhedron_3_BuilderBase<HDS>::add_vertex(point);

        OPERATION_LOG_CODE(
            cpp_cad_log::log_polygon_extrusion_builder_vertices(polygon_vertex_count, builder, vertex_count);
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }
};

}

#endif // _CPP_CAD_POLYGON_EXTRUSION_BUILDER_H
