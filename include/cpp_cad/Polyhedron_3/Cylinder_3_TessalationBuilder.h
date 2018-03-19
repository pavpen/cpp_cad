#ifndef _CPP_CAD_CYLINDER_3_TESSALATION_BUILDER_H
#define _CPP_CAD_CYLINDER_3_TESSALATION_BUILDER_H


#include <algorithm>
#include <cassert>
#include <operation_log.h>

#include "Polyhedron_3_BuilderBase.h"
#include "Cylinder_3_operation_logging.h"


namespace cpp_cad
{

// A class that uses a polyhedron incremental builer to build the faces of a
// cylinder tessalation.
template <class HDS>
class Cylinder_3_TessalationBuilder : public Polyhedron_3_BuilderBase<HDS>
{
protected:
    using Polyhedron_3_BuilderBase<HDS>::builder;

private:
    Kernel::FT base_r;
    Kernel::FT top_r;
    Kernel::FT height;
    int linear_subdivisions;
    double top_longitude;
    double top_longitude_step;
    double base_longitude;
    double base_longitude_step;
    int top_subdivision_c;
    int base_subdivision_c;
    int top_vertex_i;
    int base_vertex_i;
    int top_last_vertex_i;
    int base_last_vertex_i;

    // Difference between the current top circle vertex's longitude and the
    // current base circle vertex's longitude in subdivision counts, where the
    // subdivision denominator is top_subdivision_c * base_subdivision_c.
    int longitude_difference_subdiv;

public:
    inline Cylinder_3_TessalationBuilder(
        CGAL::Polyhedron_3<Kernel> &polyhedron, HDS& hds,
        Kernel::FT base_r = 1,
        Kernel::FT top_r = 1,
        Kernel::FT height = 1,
        int linear_subdivisions = 2)
    : base_r(base_r),
        top_r(top_r),
        height(height),
        linear_subdivisions(linear_subdivisions),
        Polyhedron_3_BuilderBase<HDS>(polyhedron, hds)
    {}

    void run()
    {
        // Subdivide each end's circle into, at least, 3 parts:
        base_subdivision_c = std::max(3, linear_subdivisions);
        top_subdivision_c = std::max(
            3, static_cast<int>( ceil(base_subdivision_c / CGAL::to_double(base_r * top_r)) ));

        int vertex_count = base_subdivision_c + top_subdivision_c;
        int face_count = base_subdivision_c + top_subdivision_c;

        // Each face has 3 halfedges.
        int halfedge_count = 3 * face_count;

        builder.begin_surface(vertex_count, face_count, halfedge_count);
        add_tessalation();
        builder.end_surface();
    }

protected:
    using Polyhedron_3_BuilderBase<HDS>::add_face;

private:
    void add_tessalation()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        base_longitude_step = 2 * M_PI / base_subdivision_c;
        top_longitude_step = 2 * M_PI / top_subdivision_c;

        // Add vertices:
        OPERATION_LOG_MESSAGE("Adding top circle vertices.");
        add_end_circle(
            CGAL::to_double(top_r), top_longitude_step, top_subdivision_c,
            CGAL::to_double(height));
        OPERATION_LOG_MESSAGE("Adding base circle vertices.");
        add_end_circle(
            CGAL::to_double(base_r), base_longitude_step, base_subdivision_c,
            0);

        base_vertex_i = top_subdivision_c;
        base_last_vertex_i = base_vertex_i + base_subdivision_c - 1;
        top_vertex_i = 0;
        top_last_vertex_i = top_vertex_i + top_subdivision_c - 1;
        add_faces();

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    void add_end_circle(double r, double longitude_step, int subdivision_c, double z)
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        double longitude = 0.0;

        for (int c = 0; c < subdivision_c; ++c, longitude += longitude_step)
        {
            add_vertex(r, longitude, z);
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_faces()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        OPERATION_LOG_DUMP_VARS(CGAL::to_double(height),
            top_subdivision_c, base_subdivision_c,
            top_vertex_i, base_vertex_i,
            top_last_vertex_i, base_last_vertex_i);

        longitude_difference_subdiv = 0;
        while (true)
        {
            // Advance to the next vertex on the top circle
            // (longitude_difference_subdiv += base_subdivision_c), or
            // advance to the next vertex on the base circle
            // (longitude_difference_subdiv -= top_subdivision_c):
            if (abs(longitude_difference_subdiv + base_subdivision_c) <
            abs(longitude_difference_subdiv - top_subdivision_c))
            {
                advance_top_vertex();
                if (top_vertex_i > top_last_vertex_i)
                {
                    complete_base();
                    break;
                }
            }
            else
            {
                advance_base_vertex();
                if (base_vertex_i > base_last_vertex_i)
                {
                    complete_top();
                    break;
                }
            }
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Add a face that connects the current and next vertices on the
    // top circle to the current vertex on the base circle.
    inline void advance_top_vertex()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        int res_next_vertex_i = top_vertex_i + 1;
        int next_vertex_i = res_next_vertex_i;

        if (res_next_vertex_i > top_last_vertex_i)
        {
            next_vertex_i -= top_subdivision_c;
        }

        assert(next_vertex_i != top_vertex_i);
        add_face(top_vertex_i, base_vertex_i, next_vertex_i);

        top_vertex_i = res_next_vertex_i;
        longitude_difference_subdiv += base_subdivision_c;

        OPERATION_LOG_DUMP_VARS(top_vertex_i, longitude_difference_subdiv);
        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Add a face that connects the current and next vertices on the
    // base circrle to the current vertex on the top circle.
    inline void advance_base_vertex()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        int res_next_vertex_i = base_vertex_i + 1;
        int next_vertex_i = res_next_vertex_i;

        if (res_next_vertex_i > base_last_vertex_i)
        {
            next_vertex_i -= base_subdivision_c;
        }

        assert(next_vertex_i != base_vertex_i);
        add_face(top_vertex_i, base_vertex_i, next_vertex_i);

        base_vertex_i = res_next_vertex_i;
        longitude_difference_subdiv -= top_subdivision_c;

        OPERATION_LOG_DUMP_VARS(base_vertex_i, longitude_difference_subdiv);
        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Connect all remaining vertices on the top circle to the current
    // vertex on the base circle by forming traiangles.
    inline void complete_top()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        assert(top_subdivision_c > 2);

        int prev_vertex_i = top_vertex_i;
        int base_vertex_i = this->base_vertex_i;

        if (base_vertex_i > base_last_vertex_i)
        {
            base_vertex_i -= base_subdivision_c;
        }

        OPERATION_LOG_DUMP_VARS(top_vertex_i);

        while (top_vertex_i <= top_last_vertex_i)
        {
            ++top_vertex_i;
            int next_vertex_i = top_vertex_i;

            if (next_vertex_i > top_last_vertex_i)
            {
                next_vertex_i -= top_subdivision_c;
            }

            OPERATION_LOG_DUMP_VARS(prev_vertex_i, base_vertex_i, top_vertex_i);

            add_face(prev_vertex_i, base_vertex_i, next_vertex_i);

            prev_vertex_i = top_vertex_i;
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    // Connect all remaining vertices on the base circle to the current
    // vertex on the top circle by forming traiangles.
    inline void complete_base()
    {
        OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

        assert(base_subdivision_c > 2);

        int prev_vertex_i = base_vertex_i;
        int top_vertex_i = this->top_vertex_i;

        if (top_vertex_i > top_last_vertex_i)
        {
            top_vertex_i -= top_subdivision_c;
        }

        OPERATION_LOG_DUMP_VARS(base_vertex_i);

        while (base_vertex_i <= base_last_vertex_i)
        {
            ++base_vertex_i;
            int next_vertex_i = base_vertex_i;

            OPERATION_LOG_DUMP_VARS(prev_vertex_i, base_vertex_i, top_vertex_i);

            if (next_vertex_i > base_last_vertex_i)
            {
                next_vertex_i -= base_subdivision_c;
            }

            add_face(prev_vertex_i, next_vertex_i, top_vertex_i);

            prev_vertex_i = base_vertex_i;
        }

        OPERATION_LOG_LEAVE_FUNCTION();
    }

    inline void add_vertex(double r, double longitude, double z)
    {
        OPERATION_LOG_ENTER_FUNCTION(r, longitude / M_PI, z);

        Kernel::Point_3 point(r * cos(longitude), r * sin(longitude), z);

        OPERATION_LOG_MESSAGE_STREAM(<<
            "Vertex " << vertex_count << ": " << point);

        builder.add_vertex(point);

        OPERATION_LOG_CODE(
            vertex_count++;

            cpp_cad_log::log_cylinder_tessalation_builder_vertices(
                CGAL::to_double(base_r), CGAL::to_double(top_r),
                CGAL::to_double(height), builder, vertex_count);
        )

        OPERATION_LOG_LEAVE_FUNCTION();
    }
};

}

#endif // _CPP_CAD_CYLINDER_3_TESSALATION_BUILDER_H
