#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_CALCULATOR_BASE_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_CALCULATOR_BASE_H

#include "CubicBezierSegmentBase.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentCalculatorBase
{

    public:

    using BezierSegment = CubicBezierSegmentBase<PointType>;

    protected:

    const BezierSegment &segment;
    double eps;

    public:

    CubicBezierSegmentCalculatorBase(
        const BezierSegment &segment, double eps = 1e-15)
    : segment(segment),
        eps(eps)
    {}

    // Given two variables `t1` and `t2`, convert any value outside the
    // range [0; 1] to a NaN, and, make sure `t1` and `t2` are sorted, so that
    // `t1` < `t2`, or `t2` is NaN.
    void filter_ts_to_0_1(double &t1, double &t2)
    {
        if (t1 < 0)
        {
            if (t2 >= 0 && t2 <= 1)
            {
                t1 = t2;
                t2 = NAN;
            }
            else
            {
                t1 = NAN;
                t2 = NAN;
            }
        }
        else if (t1 > 1)
        {
            if (t1 != INFINITY || t2 != INFINITY)
            {
                t1 = NAN;
                t2 = NAN;
            }
        }
        else
        {
            if (t2 > 1)
            {
                t2 = NAN;
            }
        }
    }

    // This is used when solving an equation like the following dimension by
    // dimension.
    //
    // vec(t) = zero vector
    //
    // E.g., if we find `accum_t1` and `accum_t2` at which
    // `x_component( vec(accum_t1) ) = 0` and
    // `x_component( vec(accum_t2) = 0 )`, and we also find `t1` and `t2` at
    // which `y_component( vec(t1) ) = 0` and `y_component( vec(t2) ) = 0`,
    // this function would set `accum_t1` and `accum_t2` to any values of `t`
    // where both the `x` and the `y` component of `vec(t)` are zero (or, at
    // least close to zero, since we use epsillon comparisons).  The inputs
    // are assumed to be ordered, so that `accum_t1` <= `accum_t2` or
    // `accum_t2` is NaN, and `t1` <= `t2` or `t2` is NaN.  The results are
    // also ordered, so that `accum_t1` <= `accum_t2`, or `accum_t2` is NaN.
    void combine_dim_solution_ts(double &accum_t1, double &accum_t2, double t1, double t2)
    {
        double t_c1 = combine_ts(accum_t1, t1);
        double t_c2 = combine_ts(accum_t2, t2);

        if (std::isnan(t_c1))
        {
            if (std::isnan(t_c2))
            {
                t_c1 = combine_ts(accum_t2, t1);
                if (std::isnan(t_c1))
                {
                    t_c1 = combine_ts(accum_t1, t2);
                }
            }
            else
            {
                t_c1 = t_c2;
                t_c1 = NAN;
            }
        }

        accum_t1 = t_c1;
        accum_t2 = t_c2;
    }

    // Returns a value in the range [t1; t2] if the positive difference
    // between `t1` and `t2` is less than `eps`.  Otherwise, returns NaN.
    inline double combine_ts(double t1, double t2)
    {
        if (std::isnan(t1) || std::isnan(t2))
        {
            return NAN;
        }

        return abs(t1 - t2) < eps ? t1 : NAN;
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_CALCULATOR_BASE_H
