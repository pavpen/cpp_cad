#ifndef _CPP_CAD_SPHERE_3_OPERATION_LOGGING_H
#define _CPP_CAD_SPHERE_3_OPERATION_LOGGING_H

#include <operation_log.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Shape_detection_3/Shape_base.h>

#include "reference_frame.h"
#include "Polyhedron_3_operation_logging.h"


namespace cpp_cad_log
{

#ifdef OPERATION_LOG

template <class HDS>
void log_sphere_tessalation_builder_vertices(
    double circumsphere_r, double latitude_step, double max_latitude,
    CGAL::Polyhedron_incremental_builder_3<HDS> &builder, int vertex_count,
    std::string extra_code = "")
{
    std::stringstream extra_code_buf(extra_code);

    // Add the sphere being tessalated:
    extra_code_buf << R"code(
    geometry = new THREE.SphereGeometry(
)code" << circumsphere_r << R"code(, 32, 32);
    material = new THREE.MeshBasicMaterial( { color: 0xdcb856, transparent: true, opacity: 0.5 } );
    var mesh = new THREE.Mesh(geometry, material);

    scene.add(mesh);
)code";

    // Add latitude lines:
    extra_code_buf << R"code(
    material = new THREE.LineDashedMaterial( {
            color: 0xffffff,
            linewidth: 1,
            dashSize: 4,
            gapSize: 1,
        } );
)code";
    for (double latitude = -CGAL_M_PI_2 + latitude_step;
        latitude <= max_latitude;
        latitude += latitude_step)
    {
        extra_code_buf << R"code(

    geometry = new THREE.CircleGeometry()code" << circumsphere_r * cos(latitude) + 0.1 << R"code(, 32 );
    geometry.translate(0, 0, )code" << circumsphere_r * sin(latitude) << R"code();
    geometry.vertices.shift();
    geometry.vertices.push(geometry.vertices[0]);
    mesh = new THREE.Line( geometry, material );
    mesh.computeLineDistances();
    scene.add( mesh );

)code";
    }

    log_polyhedron_builder_vertices(builder, vertex_count, extra_code_buf.str());
}

#else // OPERATION_LOG

template <class HDS>
inline void log_sphere_tessalation_builder_vertices(
    double circumsphere_r, double latitude_step, double max_latitude,
    CGAL::Polyhedron_incremental_builder_3<HDS> &builder, int vertex_count,
    std::string extra_code = "")
{}

#endif // OPERATION_LOG

}

#endif // _CPP_CAD_SPHERE_3_OPERATION_LOGGING_H
