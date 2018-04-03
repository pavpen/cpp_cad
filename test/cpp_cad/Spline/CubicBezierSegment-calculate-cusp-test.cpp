#include "../config.h"

#include <iostream>

#include <gtest/gtest.h>
#include <operation_log.h>

#include "cpp_cad/operation_logging/Spline/CubicBezierSegment.h"
#include "cpp_cad/reference_frame.h"
#include "cpp_cad/Spline/CubicBezierSegment.h"

namespace cpp_cad
{

namespace test
{

namespace config
{
    std::string unit_test_name = "CubicBezierSegment-calculate-cusp-test";
}

TEST(CubicBezierSegmentCuspCalculator, no_cusp_no_inflection)
{
    OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

    cpp_cad::Point_3
        p0(-10,   0, 0),
        p1( -5,  15, 0),
        p2( 20,  15, 0),
        p3( 10,   0, 0);

    cpp_cad::CubicBezierSegment<cpp_cad::Point_3>
        bezier_segment(p0, p1, p2, p3);

    double t_cusp1, t_cusp2;

    std::tie(t_cusp1, t_cusp2) = bezier_segment.calculate_cusp_ts();
    OPERATION_LOG_DUMP_VARS(t_cusp1, t_cusp2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    OPERATION_LOG_LEAVE_FUNCTION();
}


TEST(CubicBezierSegmentCuspCalculator, no_cusp_one_inflection)
{
    OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

    cpp_cad::Point_3
        p0(-10,   0, 0),
        p1(-10,  20, 0),
        p2( 10, -20, 0),
        p3( 10,   0, 0);

    cpp_cad::CubicBezierSegment<cpp_cad::Point_3>
        bezier_segment(p0, p1, p2, p3);

    double t_cusp1, t_cusp2;

    std::tie(t_cusp1, t_cusp2) = bezier_segment.calculate_cusp_ts();
    OPERATION_LOG_DUMP_VARS(t_cusp1, t_cusp2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    OPERATION_LOG_LEAVE_FUNCTION();
}


TEST(CubicBezierSegmentCuspCalculator, one_cusp)
{
    OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

    cpp_cad::Point_3
        p0(-10,  0, 0),
        p1( 10, 20, 0),
        p2(-10, 20, 0),
        p3( 10,  0, 0);

    cpp_cad::CubicBezierSegment<cpp_cad::Point_3>
        bezier_segment(p0, p1, p2, p3);

    double t_cusp1, t_cusp2;

    std::tie(t_cusp1, t_cusp2) = bezier_segment.calculate_cusp_ts();
    OPERATION_LOG_DUMP_VARS(t_cusp1, t_cusp2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    OPERATION_LOG_LEAVE_FUNCTION();
}

}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
