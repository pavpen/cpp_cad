#ifndef _CPP_CAD_POLYGON_EXTRUSION_OPERATION_LOGGING_H
#define _CPP_CAD_POLYGON_EXTRUSION_OPERATION_LOGGING_H

#include <algorithm>

#include <operation_log.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Shape_detection_3/Shape_base.h>

#include "../reference_frame.h"
#include "Polyhedron_3_operation_logging.h"

namespace cpp_cad_log
{

#ifdef OPERATION_LOG

template <class HDS>
void log_polygon_extrusion_builder_vertices(
    int polygon_vertex_count,
    CGAL::Polyhedron_incremental_builder_3<HDS> &builder, int vertex_count,
    std::string extra_code = "")
{
    std::stringstream extra_code_buf(extra_code);

    extra_code_buf << R"code(
    material = new THREE.MeshBasicMaterial( { color: 0xdcb856, transparent: true, opacity: 0.5 } );

    var line_material = new THREE.LineDashedMaterial( {
            color: 0xffffff,
            linewidth: 1,
            dashSize: 4,
            gapSize: 1,
        } );
)code";

    // Add the polygon slices that are being connected:
    for (int vertex_i = 0; vertex_i < vertex_count; )
    {
        extra_code_buf << R"code(
    var shape = new THREE.Shape([
)code";
        bool is_first = true;
        int last_vertex_i = std::min(
            vertex_count - 1, vertex_i + polygon_vertex_count - 1);

        for (; vertex_i <= last_vertex_i; ++vertex_i)
        {
            typename HDS::Vertex_handle vertex = builder.vertex(vertex_i);

            if (is_first)
            {
                is_first = false;
            }
            else
            {
                extra_code_buf << ", ";
            }
            extra_code_buf << "new THREE.Vector2(" <<
                vertex->point().x() << ", " <<
                vertex->point().y() << ", " <<
                vertex->point().z() << ")";
        }
        extra_code_buf << R"code(
    ]);
    geometry = new THREE.ShapeGeometry(shape);
    var mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);

    // Close the polygon:
    //geometry.vertices.push(geometry.vertices[0]);
    mesh = new THREE.Line( geometry, line_material );
    mesh.computeLineDistances();
    scene.add(mesh);
)code";
    }


    log_polyhedron_builder_vertices(builder, vertex_count, extra_code_buf.str());
}

#endif // OPERATION_LOG

}

#endif // _CPP_CAD_POLYGON_EXTRUSION_OPERATION_LOGGING_H
