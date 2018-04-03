#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASIS_MATRIX_CALCULATOR_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASIS_MATRIX_CALCULATOR_H

#include "../algebra/vector_3.h"
#include "CubicBezierSegmentCalculatorBase.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentBasisMatrixCalculator
: virtual protected CubicBezierSegmentCalculatorBase<PointType>
{
    public:

    using CalculatorBase = CubicBezierSegmentCalculatorBase<PointType>;
    using BezierSegment = typename CalculatorBase::BezierSegment;

    using CalculatorBase::segment;

    protected:

    // The Bézier basis matrix.  If multiplied by [t^3, t^2, t, 1], it gives
    // the location of the point on the Bézier curve at parameter t.
    double a_x, b_x, c_x, d_x;
    double a_y, b_y, c_y, d_y;
    double a_z, b_z, c_z, d_z;

    public:

    CubicBezierSegmentBasisMatrixCalculator(
        const BezierSegment &segment, double eps = 1e-15)
    : CubicBezierSegmentCalculatorBase<PointType>(segment, eps)
    {}

    void calculate_t_basis_matrix()
    {
        std::tuple<double, double, double>
            p0 = vector_3::point_to_tuple(segment.p0()),
            p1 = vector_3::point_to_tuple(segment.p1()),
            p2 = vector_3::point_to_tuple(segment.p2()),
            p3 = vector_3::point_to_tuple(segment.p3());

        std::tie(a_x, a_y, a_z) = -p0        + p1 * 3 - p2 * 3 + p3;
        std::tie(b_x, b_y, b_z) =  p0 *   3  - p1 * 6 + p2 * 3;
        std::tie(c_x, c_y, c_z) =  p0 * (-3) + p1 * 3;
        std::tie(d_x, d_y, d_z) =  p0;
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASIS_MATRIX_CALCULATOR_H
