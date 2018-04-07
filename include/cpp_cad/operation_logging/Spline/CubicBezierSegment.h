#ifndef _CPP_CAD_OPERATION_LOGGING_CUBIC_BEZIER_SEGMENT_H
#define _CPP_CAD_OPERATION_LOGGING_CUBIC_BEZIER_SEGMENT_H

#include <cmath>
#include <iterator>
#include <list>
#include <vector>

#include <operation_log.h>

#include "../../reference_frame.h"
#include "../../Spline/CubicBezierSegment.h"


namespace operation_log
{

// A value formatter for the `cpp_cad::Aff_transformation_3` data type.
template <class PointType>
class ValueFormatter<cpp_cad::CubicBezierSegment<PointType>> : public ValueFormatterI
{
    private:

    typedef cpp_cad::CubicBezierSegment<PointType> ValueType;

    const ValueType value;

    public:

    // Receives the value to format.
    ValueFormatter(const ValueType value)
    : value(value)
    {}

    // Formats values for plain text logs.
    std::string to_text() override
    {
        std::stringstream res;

        res << "CubicBezierSegment { p0=" << value.p0() <<
            ", p1=" << value.p1() <<
            ", p2=" << value.p2() <<
            ", p3=" << value.p3() << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        return to_html("");
    }

    std::string to_html(std::string extra_code)
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

    // Show vertices:
    var material = new THREE.PointsMaterial({ size: 2, vertexColors: THREE.VertexColors });
    var geometry;
    var colors;
    var vertex_color;

    addVertex = function(x, y, z)
    {
        geometry.vertices.push(new THREE.Vector3(x, y, z));
        colors.push(new THREE.Color(
            vertex_color.r, vertex_color.g, vertex_color.b));
    };
)code";

        add_cusp_points_code(code);
        add_inflection_points_code(code);
        add_control_points_code(code);
        add_curve_code(code);
        add_linerization_code(code);

        code << R"code(
    // Register the scene for rendering:
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

        return code.str();
    }

    private:

    void add_cusp_points_code(std::stringstream &code)
    {
        double t_cusp1, t_cusp2;

        std::tie(t_cusp1, t_cusp2) = value.calculate_cusp_ts();

        if (!std::isnan(t_cusp1) && std::isfinite(t_cusp1))
        {
            code << R"code(
    // Show cusp points:
    vertex_color = { "r": 0.85, "g": 0.15, "b": 0.15 };
    material = new THREE.PointsMaterial({ size: 2, vertexColors: THREE.VertexColors });
    geometry = new THREE.Geometry();
    colors = [];
)code";
            cpp_cad::Point_3 point = value.evaluate(t_cusp1);
            code << "addVertex(" <<
                point.x() << ", " <<
                point.y() << ", " <<
                point.z() << ");" << std::endl;
            if (!std::isnan(t_cusp2) && std::isfinite(t_cusp2))
            {
                point = value.evaluate(t_cusp2);
                code << "addVertex(" <<
                    point.x() << ", " <<
                    point.y() << ", " <<
                    point.z() << ");" << std::endl;
            }
            code << R"code(
    geometry.colors = colors;
    geometry.computeBoundingBox();

    var pointCloud = new THREE.Points(geometry, material);

    scene.add(pointCloud);
)code";
        }
    }

    void add_inflection_points_code(std::stringstream &code)
    {
        double t_inflection1, t_inflection2;

        std::tie(t_inflection1, t_inflection2) = value.calculate_inflection_ts();

        if (!std::isnan(t_inflection1) && std::isfinite(t_inflection1))
        {
            code << R"code(
    // Show inflection points:
    vertex_color = { "r": 0.15, "g": 0.85, "b": 0.15 };
    material = new THREE.PointsMaterial({ size: 2, vertexColors: THREE.VertexColors });
    geometry = new THREE.Geometry();
    colors = [];
)code";
            cpp_cad::Point_3 point = value.evaluate(t_inflection1);
            code << "addVertex(" <<
                point.x() << ", " <<
                point.y() << ", " <<
                point.z() << ");" << std::endl;
            if (!std::isnan(t_inflection2) && std::isfinite(t_inflection2))
            {
                point = value.evaluate(t_inflection2);
                code << "addVertex(" <<
                    point.x() << ", " <<
                    point.y() << ", " <<
                    point.z() << ");" << std::endl;
            }
            code << R"code(
    geometry.colors = colors;
    geometry.computeBoundingBox();

    var pointCloud = new THREE.Points(geometry, material);

    scene.add(pointCloud);
)code";
        }
    }

    void add_control_points_code(std::stringstream &code)
    {
        code << R"code(
    // Show control points:
    geometry = new THREE.Geometry();
    colors = [];
    vertex_color = { "r": 0.15, "g": 0.15, "b": 0.85 };

)code";

        std::vector<cpp_cad::Point_3> control_points =
            { value.p0(), value.p1(), value.p2(), value.p3() };

        for (const auto &point : control_points)
        {
            code << "    addVertex(" <<
                point.x() << ", " <<
                point.y() << ", " <<
                point.z() << ");" << std::endl;
        }

        code << R"code(

    geometry.colors = colors;
    geometry.computeBoundingBox();

    var pointCloud = new THREE.Points(geometry, material);

    scene.add(pointCloud);
)code";
    }

    void add_curve_code(std::stringstream &code)
    {
        code << R"code(
    // Show the Bézier segment:
    material = new THREE.LineBasicMaterial( { color : 0xd0d0d0 } );

    var curve = new THREE.CubicBezierCurve3(
        geometry.vertices[0], geometry.vertices[1],
        geometry.vertices[2], geometry.vertices[3]);

    geometry = new THREE.BufferGeometry().setFromPoints( curve.getPoints(50) );

    var curveObject = new THREE.Line( geometry, material );

    scene.add(curveObject);
)code";
    }

    void add_linerization_code(std::stringstream &code)
    {
        code << R"code(
    // Draw the curve linearization:
    material = new THREE.LineBasicMaterial( { color : 0xa0d0a0 } );

    geometry = new THREE.BufferGeometry();
)code";
        std::list<double> vertex_ts;
        value.linearize_ts(std::back_inserter(vertex_ts), 1);
        for (const auto t : vertex_ts)
        {
            cpp_cad::Point_3 point = value.evaluate(t);

            code << "    geometry.vertices.push(new THREE.Vector3(" <<
                point.x() << ", " << point.y() << ", " << point.z() <<
                "));" << std::endl;
        }
        code << R"code(
    geometry.computeBoundingBox();

    curveObject = new THREE.Line( geometry, material );

    scene.add(curveObject);
)code";
    }
};


}

#endif // _CPP_CAD_OPERATION_LOGGING_CUBIC_BEZIER_SEGMENT_H
