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
    std::string unit_test_name = "CubicBezierSegment-cusp-and-inflection-calculator-test";
}

TEST(CubicBezierSegmentCuspAndInflectionCalculator, no_cusp_no_inflection)
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

    double t_inflect1, t_inflect2;

    std::tie(t_inflect1, t_inflect2) =
        bezier_segment.calculate_inflection_ts();
    OPERATION_LOG_DUMP_VARS(t_inflect1, t_inflect2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    ASSERT_TRUE(std::isnan(t_cusp1));
    ASSERT_TRUE(std::isnan(t_cusp2));
    ASSERT_TRUE(std::isnan(t_inflect1));
    ASSERT_TRUE(std::isnan(t_inflect2));

    OPERATION_LOG_LEAVE_FUNCTION();
}


TEST(CubicBezierSegmentCuspAndInflectionCalculator, no_cusp_one_inflection)
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

    double t_inflect1, t_inflect2;

    std::tie(t_inflect1, t_inflect2) =
        bezier_segment.calculate_inflection_ts();
    OPERATION_LOG_DUMP_VARS(t_inflect1, t_inflect2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    ASSERT_TRUE(std::isnan(t_cusp1));
    ASSERT_TRUE(std::isnan(t_cusp2));
    ASSERT_NEAR(t_inflect1, 0.5, 1e-15);
    ASSERT_TRUE(std::isnan(t_inflect2));

    OPERATION_LOG_LEAVE_FUNCTION();
}


TEST(CubicBezierSegmentCuspAndInflectionCalculator, one_cusp_no_inflection)
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

    double t_inflect1, t_inflect2;

    std::tie(t_inflect1, t_inflect2) =
        bezier_segment.calculate_inflection_ts();
    OPERATION_LOG_DUMP_VARS(t_inflect1, t_inflect2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    ASSERT_NEAR(t_cusp1, 0.5, 1e-15);
    ASSERT_TRUE(std::isnan(t_cusp2));
    ASSERT_TRUE(std::isnan(t_inflect1));
    ASSERT_TRUE(std::isnan(t_inflect2));

    OPERATION_LOG_LEAVE_FUNCTION();
}


TEST(CubicBezierSegmentCuspAndInflectionCalculator, degenerate)
{
    OPERATION_LOG_ENTER_NO_ARG_FUNCTION();

    cpp_cad::Point_3
        p0(-10,  0, 0),
        p1(-10,  0, 0),
        p2(-10,  0, 0),
        p3(-10,  0, 0);

    cpp_cad::CubicBezierSegment<cpp_cad::Point_3>
        bezier_segment(p0, p1, p2, p3);

    double t_cusp1, t_cusp2;

    std::tie(t_cusp1, t_cusp2) = bezier_segment.calculate_cusp_ts();
    OPERATION_LOG_DUMP_VARS(t_cusp1, t_cusp2);

    double t_inflect1, t_inflect2;

    std::tie(t_inflect1, t_inflect2) =
        bezier_segment.calculate_inflection_ts();
    OPERATION_LOG_DUMP_VARS(t_inflect1, t_inflect2);

    OPERATION_LOG_DUMP_VARS(bezier_segment);

    ASSERT_TRUE(t_cusp1 == INFINITY);
    ASSERT_TRUE(t_cusp2 == INFINITY);
    ASSERT_TRUE(t_inflect1 == INFINITY);
    ASSERT_TRUE(t_inflect2 == INFINITY);

    OPERATION_LOG_LEAVE_FUNCTION();
}

}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
