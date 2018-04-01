#ifndef _CPP_CAD_OPERATION_LOGGING_POLYGON_2_TRANSFORM_ITERATOR_H
#define _CPP_CAD_OPERATION_LOGGING_POLYGON_2_TRANSFORM_ITERATOR_H

#include <operation_log.h>

#include "../../TransformIterator/Polygon_2_TransformIterator.h"


namespace operation_log
{

// A value formatter for a `cpp_cad::Polygon_2_TransformIterator` data type.
template <class TransformIterator>
class ValueFormatter<cpp_cad::Polygon_2_TransformIterator<TransformIterator>> : public ValueFormatterI
{
    private:

    using ValueType = cpp_cad::Polygon_2_TransformIterator<TransformIterator>;

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
            ValueFormatter<TransformIterator>(value.transform_iterator).to_text() << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        return HtmlUtils::escape(to_text());
    }
};

}

#endif // _CPP_CAD_OPERATION_LOGGING_POLYGON_2_TRANSFORM_ITERATOR_H
