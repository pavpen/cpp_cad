#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_CUSP_CALCULATOR_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_CUSP_CALCULATOR_H

#include <tuple>

#include "CubicBezierSegment.h"
#include "CubicBezierSegmentBasisMatrixCalculator.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentCuspCalculator
: virtual public CubicBezierSegmentBasisMatrixCalculator<PointType>
{
    public:

    // C++ compilers need a lot of help:
    using CalculatorBase = CubicBezierSegmentCalculatorBase<PointType>;
    using BezierSegment = typename CalculatorBase::BezierSegment;
    using BasisMatrixCalculator =
        CubicBezierSegmentBasisMatrixCalculator<PointType>;

    using BasisMatrixCalculator::a_x;
    using BasisMatrixCalculator::b_x;
    using BasisMatrixCalculator::c_x;
    using BasisMatrixCalculator::a_y;
    using BasisMatrixCalculator::b_y;
    using BasisMatrixCalculator::c_y;
    using BasisMatrixCalculator::a_z;
    using BasisMatrixCalculator::b_z;
    using BasisMatrixCalculator::c_z;
    using BasisMatrixCalculator::eps;
    using BasisMatrixCalculator::calculate_t_basis_matrix;
    using CalculatorBase::combine_dim_solution_ts;
    using CalculatorBase::filter_ts_to_0_1;

    CubicBezierSegmentCuspCalculator(
        const BezierSegment &segment, double eps = 1e-15)
    : CalculatorBase(segment, eps),
        BasisMatrixCalculator(segment, eps)
    {}

    // Returns times of cusps in the [0; 1] range.
    //
    // Requires the Bézier basis matrix to have been calculated.
    std::tuple<double, double> calculate_cusp_ts()
    {
        double t_cusp1, t_cusp2;
        double t1, t2;

        // Find where the velocity vector is zero in the x dimension:
        std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_x, b_x, c_x);

        // Find where V_y = 0:
        if (t_cusp1 == INFINITY && t_cusp2 == INFINITY)
        {
            std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_y, b_y, c_y);
        }
        else
        {
            std::tie(t1, t2) = calculate_dim_cusp_ts(a_y, b_y, c_y);
            if (t1 != INFINITY || t2 != INFINITY)
            {
                combine_dim_solution_ts(t_cusp1, t_cusp2, t1, t2);
            }
        }

        // Find where V_z = 0:
        if (t_cusp1 == INFINITY && t_cusp2 == INFINITY)
        {
            std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_z, b_z, c_z);
        }
        else
        {
            std::tie(t1, t2) = calculate_dim_cusp_ts(a_z, b_z, c_z);
            if (t1 != INFINITY || t2 != INFINITY)
            {
                combine_dim_solution_ts(t_cusp1, t_cusp2, t1, t2);
            }
        }

        // Only accept cusps in the [0; 1] range:
        filter_ts_to_0_1(t_cusp1, t_cusp2);

        return std::make_tuple(t_cusp1, t_cusp2);
    }

    protected:

    // Calculates the values of the `t` parameter at which cusps occur along
    // a given dimension.  I.e., when the velocity vector becomes zero in that
    // dimension.
    //
    // If a velocity vector becomes zero in all dimensions at a given `t`,
    // then there's a cusp at that `t`.
    //
    // `a`, `b`, `c` are the first 3 components of the Bézier basis matrix
    // above for the demnsion along which we are testing the velocity vector
    // for a zero magnitude.
    //
    // The possible results are:
    //     * All values of `t`, returned as (+infinity, +infinity).
    //     * Two values of `t`, returned as (double, double >= first double).
    //     * One value of `t`,  returned as (double, NaN).
    //     * No cusps, returned as (NaN, NaN).
    std::tuple<double, double> calculate_dim_cusp_ts(
        double a, double b, double c)
    {
        // V = 3 * a * t^2 + 2 * b * t + c
        if (fabs(a) < eps)
        {
            // V = 2 * b * t + c
            if (fabs(b) < eps)
            {
                // V = c
                if (fabs(c) < eps)
                {
                    // V = 0
                    return std::make_tuple(INFINITY, INFINITY);
                }
                else
                {
                    // V != 0
                    return std::make_tuple(NAN, NAN);
                }
            }
            else
            {
                // V = 2 * b * t + c, b != 0:
                return std::make_tuple(
                    -c / (2 * b),
                    NAN);
            }
        }
        else
        {
            // V = 3 * a * t^2 + 2 * b * t + c, a != 0:
            double det = sqrt(b * b - 3 * a * c);

            if (det < eps)
            {
                // Coinciding quadratic solutions. Report 1 solution.
                return std::make_tuple(-b / (3 * a), NAN);
            }
            else
            {
                // solution 2 != solution 1.
                // Order solutions, so t1 < t2:
                double t1, t2;

                if (a < 0)
                {
                    t1 = (-b + det) / (3 * a);
                    t2 = (-b - det) / (3 * a);
                }
                else
                {
                    t1 = (-b - det) / (3 * a);
                    t2 = (-b + det) / (3 * a);
                }

                return std::make_tuple(t1, t2);
            }
        }
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_CUSP_CALCULATOR_H
