#ifndef _CPP_CAD_OPERATION_LOGGING_ROTATE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_OPERATION_LOGGING_ROTATE_TRANSFORM_ITERATOR_H

#include <operation_log.h>

#include "../../Aff_transformation_3.h"
#include "../../TransformIterator/RotateTransformIterator.h"


namespace operation_log
{

// A value formatter for a `cpp_cad::RotateTransformIterator` data type.
template <cpp_cad::Aff_transformation_3 TransformFunctor(double angle)>
class ValueFormatter<cpp_cad::RotateTransformIterator<TransformFunctor>> : public ValueFormatterI
{
    private:

    using ValueType = cpp_cad::RotateTransformIterator<TransformFunctor>;

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

        res << "RotateTransformIterator { angle=" << value.angle <<
            ", angle_step=" << value.angle_step <<
            ", subdivision_i=" << value.subdivision_i << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        return HtmlUtils::escape(to_text());
    }
};


}

#endif // _CPP_CAD_OPERATION_LOGGING_ROTATE_TRANSFORM_ITERATOR_H
