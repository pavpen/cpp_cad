#ifndef _CPP_CAD_POLYHEDRON_3_OPERATION_LOGGING_H
#define _CPP_CAD_POLYHEDRON_3_OPERATION_LOGGING_H

#include <operation_log.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Shape_detection_3/Shape_base.h>

#include "../reference_frame.h"

namespace cpp_cad_log
{

template <class HDS>
void log_polyhedron_builder_vertices(
    CGAL::Polyhedron_incremental_builder_3<HDS> &builder, int vertex_count,
    std::string extra_code = "")
{
    std::stringstream code;

    code << R"code(
<script type="text/javascript">

(function ()
{
    var camera = new THREE.PerspectiveCamera(70, 500 / 500, 0.01, 1000);

    camera.position.z = 50;

    var scene = new THREE.Scene();

    var axesHelper = new THREE.AxesHelper( 20 );
    scene.add( axesHelper );

    var material = new THREE.PointsMaterial({ size: 2, vertexColors: THREE.VertexColors });
    var geometry = new THREE.Geometry();
    var colors = [];

    addVertex = function(x, y, z)
    {
        geometry.vertices.push(new THREE.Vector3(x, y, z));
        colors.push(new THREE.Color(0.15, 0.15, 0.85));
    };
)code";

    for (int i = 0; i < vertex_count; ++i)
    {
        typename HDS::Vertex_handle vertex = builder.vertex(i);

        code << "    addVertex(" <<
            vertex->point().x() << ", " <<
            vertex->point().y() << ", " <<
            vertex->point().z() << ");" << std::endl;
    }

        code <<
R"code(

    geometry.colors = colors;
    geometry.computeBoundingBox();

    var pointCloud = new THREE.Points(geometry, material);

    scene.add(pointCloud);

    var renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(250, 250);
    renderer.autoClear = false;

    var sceneView = new operation_log_3js.SceneView(scene, renderer, camera);

    sceneView.labelVertices(pointCloud);

    var controls = new THREE.TrackballControls( camera );

    controls.rotateSpeed = 1.0;
    controls.zoomSpeed = 1.2;
    controls.panSpeed = 0.8;
    controls.noZoom = false;
    controls.noPan = false;
    controls.staticMoving = true;
    controls.dynamicDampingFactor = 0.3;
    controls.keys = [ 65, 83, 68 ];
    controls.addEventListener('change', function()
        {
            sceneView.render();
        });
    
    sceneView.controls = controls;

    sceneView.animationFrame = function(time) {
            sceneView.render();
        };
)code" <<
        extra_code <<
R"code(
    operation_log_3js.document.addSceneView(sceneView);
})();

</script>    
)code";

    operation_log::OperationLogInstance::get().write_html(code.str());
}

}

#endif // _CPP_CAD_POLYHEDRON_3_OPERATION_LOGGING_H
