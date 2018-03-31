#include <iostream>

#include <gtest/gtest.h>

#include "cpp_cad/reference_frame.h"
#include "cpp_cad/Spline/CubicBezierSegment.h"

namespace
{

TEST(CubicBezierSegmentCuspCalculator, calculate_cusp_ts)
{
    cpp_cad::Point_3
        p0(-1, 0, 0),
        p1( 1, 2, 0),
        p2( 0, 2, 0),
        p3( 1, 0, 0);

    cpp_cad::CubicBezierSegment<cpp_cad::Point_3>
        bezier_segment(p0, p1, p2, p3);

    double t_cusp1, t_cusp2;

    std::tie(t_cusp1, t_cusp2) = bezier_segment.calculate_cusp_ts();
    std::cout << "Cusp `t`s: " << "(" << t_cusp1 << ", " << t_cusp2 << ")" <<
        std::endl;
}

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
