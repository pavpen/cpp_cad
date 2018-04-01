#ifndef _CPP_CAD_POLYHEDRON_3_OPERATION_LOGGING_H
#define _CPP_CAD_POLYHEDRON_3_OPERATION_LOGGING_H

#include <operation_log.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Shape_detection_3/Shape_base.h>

#include "../reference_frame.h"

namespace operation_log
{

// A value formatter for the `cpp_cad::Polygon_2` data type.
template <>
class ValueFormatter<cpp_cad::Polygon_2> : public ValueFormatterI
{
    private:

    const cpp_cad::Polygon_2 value;

    public:

    // Receives the value to format.
    ValueFormatter(const cpp_cad::Polygon_2 value)
    : value(value)
    {}

    // Formats values for plain text logs.
    std::string to_text() override
    {
        std::stringstream res;

        res << "Polygon_2 { ";
        typename cpp_cad::Polygon_2::Vertex_const_iterator vit;
        bool is_first = true;

        for (vit = value.vertices_begin(); vit != value.vertices_end(); ++vit)
        {
            if (is_first)
            {
                is_first = false;
            }
            else
            {
                res << ", ";
            }
            res << (*vit);
        }
        res << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        // We can output a Three.js scene, or an SVG element here.
        return HtmlUtils::escape(to_text());
    }
};


// A value formatter for the `cpp_cad::Polygon_2` data type.
template <class TransformIterator>
class ValueFormatter<cpp_cad::Polygon_2_TransformIterator<TransformIterator>> : public ValueFormatterI
{
    private:

    typedef cpp_cad::Polygon_2_TransformIterator<TransformIterator> ValueType;

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

        res << "Polygon_2_TransformIterator { transform_iterator=" <<
            value.transform_iterator << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        return HtmlUtils::escape(to_text());
    }
};

// A value formatter for the `cpp_cad::Polygon_2` data type.
template <>
class ValueFormatter<cpp_cad::Aff_transformation_3> : public ValueFormatterI
{
    private:

    typedef cpp_cad::Aff_transformation_3 ValueType;

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

        res << "Aff_transformation_3 { " << std::endl <<
            "| " << std::setw(5) << value.m(0, 0) << " " <<
                std::setw(5) << value.m(0, 1) << " " <<
                std::setw(5) << value.m(0, 2) << " " <<
                std::setw(5) << value.m(0, 3) << " |" << std::endl <<
            "| " << std::setw(5) << value.m(1, 0) << " " <<
                std::setw(5) << value.m(1, 1) << " " <<
                std::setw(5) << value.m(1, 2) << " " <<
                std::setw(5) << value.m(1, 3) << " |" << std::endl <<
            "| " << std::setw(5) << value.m(2, 0) << " " <<
                std::setw(5) << value.m(2, 1) << " " <<
                std::setw(5) << value.m(2, 2) << " " <<
                std::setw(5) << value.m(2, 3) << " |" << std::endl <<
            "| " << std::setw(5) <<         0 << " " <<
                std::setw(5) <<             0 << " " <<
                std::setw(5) <<             0 << " " <<
                std::setw(5) <<             1 << " |" << std::endl <<
            "}";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        std::stringstream res;

        res << R"code(<span>Aff_transformation_3:</span>
    <math class="operation-log-matrix">
        <mrow>
            <mo>[</mo>
            <mtable>
                <mtr>
                    <mtd><mi>)code" << value.m(0, 0) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(0, 1) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(0, 2) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(0, 3) << R"code(</mi></mtd>
                </mtr>
                <mtr>
                    <mtd><mi>)code" << value.m(1, 0) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(1, 1) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(1, 2) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(1, 3) << R"code(</mi></mtd>
                </mtr>
                <mtr>
                    <mtd><mi>)code" << value.m(2, 0) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(2, 1) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(2, 2) << R"code(</mi></mtd>
                    <mtd><mi>)code" << value.m(2, 3) << R"code(</mi></mtd>
                </mtr>
                <mtr>
                    <mtd><mi>0</mi></mtd>
                    <mtd><mi>0</mi></mtd>
                    <mtd><mi>0</mi></mtd>
                    <mtd><mi>1</mi></mtd>
                </mtr>
            </mtable>
            <mo>]</mo>
        </mrow>
    </math>
)code" << std::endl;

        return res.str();
    }
};

}

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
