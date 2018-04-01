#ifndef _CPP_CAD_OPERATION_LOGGING_POLYGON_2_H
#define _CPP_CAD_OPERATION_LOGGING_POLYGON_2_H

#include <operation_log.h>

#include "../Polygon_2.h"


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


}

#endif // _CPP_CAD_OPERATION_LOGGING_POLYGON_2_H
