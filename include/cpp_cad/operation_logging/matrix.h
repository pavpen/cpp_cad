#ifndef _CPP_CAD_OPERATION_LOGGING_MATRIX_H
#define _CPP_CAD_OPERATION_LOGGING_MATRIX_H

#include <vector>


namespace cpp_cad
{

namespace log
{

template <typename ScalarType>
std::string format_matrix_to_text(int column_count, std::vector<ScalarType> values)
{
    std::stringstream res;
    int column_i = 0;

    for (auto value_iter = values.cbegin(); value_iter != values.cend(); )
    {
        res << "| ";
        for (int column_i = 0; column_i < column_count; ++column_i)
        {
            res << std::setw(5) << (*value_iter) << " ";
            ++value_iter;
        }
        res << " |" << std::endl;
    }

    return res.str();
}

template <typename ScalarType>
std::string format_matrix_to_html(int column_count, std::vector<ScalarType> values)
{
    std::stringstream res;
    int column_i = 0;

    res << R"code(
    <math class="operation-log-matrix">
        <mrow>
            <mo>[</mo>
            <mtable>
)code";
    for (auto value_iter = values.cbegin(); value_iter != values.cend(); )
    {
        res << R"code(
                <mtr>
            )code";
        for (int column_i = 0; column_i < column_count; ++column_i)
        {
            res << R"code(
                    <mtd><mi>)code" << (*value_iter) << R"code(</mi></mtd>
)code";
            ++value_iter;
        }
        res << R"code(
                </mtr>
)code";
    }

    res << R"code(
            </mtable>
            <mo>]</mo>
        </mrow>
    </math>
)code";

    return res.str();
}

}

}

#endif // _CPP_CAD_OPERATION_LOGGING_MATRIX_H
