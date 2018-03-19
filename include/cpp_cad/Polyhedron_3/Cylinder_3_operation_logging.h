#ifndef _CPP_CAD_CYLINDER_3_OPERATION_LOGGING_H
#define _CPP_CAD_CYLINDER_3_OPERATION_LOGGING_H

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
void log_cylinder_tessalation_builder_vertices(
    double base_r, double top_r, double height,
    CGAL::Polyhedron_incremental_builder_3<HDS> &builder, int vertex_count,
    std::string extra_code = "")
{
    std::stringstream extra_code_buf(extra_code);

    // Add the cylinder being tessalated:
    extra_code_buf << R"code(
    geometry = new THREE.CylinderGeometry(
)code" << top_r << ", " << base_r << ", " << height << R"code(, 32);
    geometry.rotateX(Math.PI / 2);
    geometry.translate(0, 0, )code" << height / 2 << R"code();
    material = new THREE.MeshBasicMaterial( { color: 0xdcb856, transparent: true, opacity: 0.5 } );
    var mesh = new THREE.Mesh(geometry, material);

    scene.add(mesh);

    // Add base and top circles:
    material = new THREE.LineDashedMaterial( {
            color: 0xffffff,
            linewidth: 1,
            dashSize: 4,
            gapSize: 1,
        } );

    // Top circle:
    geometry = new THREE.CircleGeometry()code" << top_r + 0.1 << R"code(, 32 );
    geometry.translate(0, 0, )code" << height << R"code();
    geometry.vertices.shift();
    geometry.vertices.push(geometry.vertices[0]);
    mesh = new THREE.Line( geometry, material );
    mesh.computeLineDistances();
    scene.add( mesh );

    // Base circle:
    geometry = new THREE.CircleGeometry()code" << base_r + 0.1 << R"code(, 32 );
    geometry.vertices.shift();
    geometry.vertices.push(geometry.vertices[0]);
    mesh = new THREE.Line( geometry, material );
    mesh.computeLineDistances();
    scene.add( mesh );

)code";

    log_polyhedron_builder_vertices(builder, vertex_count, extra_code_buf.str());
}

#endif // OPERATION_LOG

}

#endif // _CPP_CAD_CYLINDER_3_OPERATION_LOGGING_H
