#ifndef _CPP_CAD_OPERATION_LOGGING_AFF_TRANSFORMATION_3_H
#define _CPP_CAD_OPERATION_LOGGING_AFF_TRANSFORMATION_3_H

#include <operation_log.h>

#include "../Aff_transformation_3.h"


namespace operation_log
{

// A value formatter for the `cpp_cad::Aff_transformation_3` data type.
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

#endif // _CPP_CAD_OPERATION_LOGGING_AFF_TRANSFORMATION_3_H
