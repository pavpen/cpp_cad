#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_INFLECTION_CALCULATOR_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_INFLECTION_CALCULATOR_H

#include <tuple>

#include "CubicBezierSegment.h"
#include "CubicBezierSegmentBasisMatrixCalculator.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentInflectionCalculator
: virtual public CubicBezierSegmentBasisMatrixCalculator<PointType>
{
    public:

    // C++ compilers need a lot of help:
    using CalculatorBase = CubicBezierSegmentCalculatorBase<PointType>;
    using BezierSegment = typename CalculatorBase::BezierSegment;
    using BasisMatrixCalculator =
        CubicBezierSegmentBasisMatrixCalculator<PointType>;

    using BasisMatrixCalculator::a_x;
    using BasisMatrixCalculator::a_y;
    using BasisMatrixCalculator::a_z;
    using BasisMatrixCalculator::b_x;
    using BasisMatrixCalculator::b_y;
    using BasisMatrixCalculator::b_z;
    using BasisMatrixCalculator::c_x;
    using BasisMatrixCalculator::c_y;
    using BasisMatrixCalculator::c_z;
    using BasisMatrixCalculator::eps;
    using CalculatorBase::combine_dim_solution_ts;
    using CalculatorBase::filter_ts_to_0_1;

    CubicBezierSegmentInflectionCalculator(
        const BezierSegment &segment, double eps = 1e-15)
    : CalculatorBase(segment, eps),
        BasisMatrixCalculator(segment, eps)
    {}

    // Returns `t` parameter values in the [0; 1] range at which inflection
    // points occur.
    //
    // Requires the Bézier basis matrix to have been calculated.
    std::tuple<double, double> calculate_inflection_ts()
    {
        double t_inflect1, t_inflect2;
        double t1, t2;

        // Find where the cross product is zero in the x dimension:
        std::tie(t_inflect1, t_inflect2) =
            calculate_dim_inflection_ts(a_y, a_z, b_y, b_z, c_y, c_z);

        // Find where the product is zero in the y dimension:
        if (t_inflect1 == INFINITY && t_inflect2 == INFINITY)
        {
            std::tie(t_inflect1, t_inflect2) =
                calculate_dim_inflection_ts(a_z, a_x, b_z, b_x, c_z, c_x);
        }
        else
        {
            std::tie(t1, t2) =
                calculate_dim_inflection_ts(a_z, a_x, b_z, b_x, c_z, c_x);
            combine_dim_solution_ts(t_inflect1, t_inflect2, t1, t2);
        }

        // Find 0 in the z dimension:
        if (t_inflect1 == INFINITY && t_inflect2 == INFINITY)
        {
            std::tie(t_inflect1, t_inflect2) =
                calculate_dim_inflection_ts(a_x, a_y, b_x, b_y, c_x, c_y);
        }
        else
        {
            std::tie(t1, t2) =
                calculate_dim_inflection_ts(a_x, a_y, b_x, b_y, c_x, c_y);
            combine_dim_solution_ts(t_inflect1, t_inflect2, t1, t2);
        }

        // Only accept inflection points in the [0; 1] range:
        filter_ts_to_0_1(t_inflect1, t_inflect2);

        return std::make_tuple(t_inflect1, t_inflect2);
    }

    protected:

    // Calculates the `t` parameter at which inflection points occur along a
    // given dimension.  I.e., when the cross product between the
    // velocity vector and the acceleration vector is zero along that
    // dimension.
    //
    // If the cross product is zero in all dimensions for some `t`, then the
    // velocity vector and the acceleration vector a collinear, and there's an
    // inflection point at that `t`.
    //
    // `d` = dimension index.  One of {`x`, `y`, `z`}.
    // `e` = the next dimension after `d`.  The dimension after `x` is `y`.
    //       The dimension after `y` is `z`.  The dimension after `z` is `x`.
    //
    // `a_d`, `a_e` = elements from the `a` column of the Bézier basis matrix
    //         for dimensions `d` and `e`.
    // `b_d`, `b_e` = elements from the `b` column of the Bézier basis matrix
    //         for dimensions `d` and `e`.
    // `c_d`, `c_e` = elemenst from the `c` column of the matrix for
    //         dimensions `d` and `e`.
    //
    // The possible results are:
    //     * All values of `t`, returned as (+infinity, +infinity).
    //     * Two values of `t`, returned as (double, double >= first double).
    //     * One value of `t`,  returned as (double, NaN).
    //     * No inflection points, returned as (NaN, NaN).
    std::tuple<double, double> calculate_dim_inflection_ts(
        double a_d, double a_e, double b_d, double b_e, double c_d, double c_e)
    {
        double t_squared_coeff = a_e * b_d - a_d * b_e;
        double t_coeff = a_e * c_d - a_d * c_e;
        double const_coeff = b_e * c_d - b_d * c_e;

        if (fabs(t_squared_coeff) < eps)
        {
            // t_squared_coeff = 0, this is a linear equation.
            if (fabs(t_coeff) < eps)
            {
                // t_coeff is also = 0, this is const = 0 equation.
                if (fabs(const_coeff) < eps)
                {
                    // This is a 0 = 0 equation:
                    return std::make_tuple(INFINITY, INFINITY);
                }
                else
                {
                    // This is a non-zero = 0 equation:
                    return std::make_tuple(NAN, NAN);
                }
            }
            else
            {
                return std::make_tuple(-(const_coeff / t_coeff) / 3, NAN);
            }
        }
        else
        {
            double t_cusp =
                -0.5 * ( t_coeff / t_squared_coeff );
            double det =
                sqrt( t_cusp * t_cusp - 1/3 * const_coeff / t_squared_coeff );

            return std::make_tuple(
                t_cusp - det,
                det < eps ?
                    NAN : // solution 2 = solution 1, report one solution.
                    t_cusp + det ); // solution 2 != solution 1.
        }
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_INFLECTION_CALCULATOR_H
