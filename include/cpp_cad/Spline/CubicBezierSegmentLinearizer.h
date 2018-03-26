#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include "CubicBezierSegment.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentLinearizer;

// Uses the algorithm described by Hain, Ahmad, and Langan in
// <https://pdfs.semanticscholar.org/8963/c06a92d6ca8868348b0930bbb800ff6e7920.pdf>
// to split a cubic Bézier curve segment into linear segments.
template <class PointType>
class CubicBezierSegmentLinearizer
{
    private:

    const CubicBezierSegment<PointType> &segment;

    // The Bézier basis matrix.  If multiplied by [t^3, t^2, t, 1], it gives
    // the location of the point on the Bézier curve at parameter t.
    double a_x, b_x, c_x, d_x;
    double a_y, b_y, c_y, d_y;
    double a_z, b_z, c_z, d_z;

    // Values of the `t` parameter at which inflection points occur.  (I.e.,
    // when the velocity and the acceleration vectors become collinear.)
    double t_inflect1, t_inflect2;

    // When two floating-point values are closer to each other than `eps`,
    // they are considered equal (enough).
    //
    // Set this to something close to but higher than the minimum precision.
    double eps;

    // Values of the `t` parameter at which cusps occur.  (I.e., when the
    // velocity vector becomes a zero vector.)
    //
    // The values are:
    //     * Velocity is always zero: t1_cusp = +infinity, t2_cusp = +infinity.
    //     * Two cusps: t_cusp1 in [0; 1], t_cusp2 in [0; 1].
    //     * One cusp: t_cusp1 in [0; 1], t_cusp2 = NaN.
    //     * No cusps: t_cusp1 = NaN, t_cusp2 = NaN.
    double t_cusp1, t_cusp2;

    public:

    CubicBezierSegmentLinearizer(
        const CubicBezierSegment<PointType> &segment, double eps = 1e-15)
    : segment(segment),
        eps(eps)
    {}

    // Returns the values of the `t` parameter for the points on the curve
    // that can be connected by line segments.
    //
    // `max_lateral_distance` = The `f` parameter in the Hain, Ahmad, Langan
    //         paper.  I.e. an estimate of the maximum (lateral) distance of a
    //         line segment from the curve.
    std::vector<double> linearize_ts(double max_lateral_distance)
    {
        calculate_t_basis_matix();
        calculate_cusp_ts();
        calculate_inflection_ts();
        throw "TODO: Not implemented!";
    }

    private:

    void calculate_t_basis_matix()
    {
        a_x = -segment.p0().x() + 3 * segment.p1().x() - 3 * segment.p2().x() + segment.p3().x();
        a_y = -segment.p0().y() + 3 * segment.p1().y() - 3 * segment.p2().y() + segment.p3().y();
        a_z = -segment.p0().z() + 3 * segment.p1().z() - 3 * segment.p2().z() + segment.p3().z();

        b_x = 3 * segment.p0().x() - 6 * segment.p1().x() + 3 * segment.p2().x();
        b_y = 3 * segment.p0().y() - 6 * segment.p1().y() + 3 * segment.p2().y();
        b_z = 3 * segment.p0().z() - 6 * segment.p1().z() + 3 * segment.p2().z();

        c_x = -3 * segment.p0().x() + 3 * segment.p1().x();
        c_y = -3 * segment.p0().y() + 3 * segment.p1().y();
        c_z = -3 * segment.p0().z() + 3 * segment.p1().z();

        d_x = segment.p0().x();
        d_y = segment.p0().y();
        d_z = segment.p0().z();
    }

    // Calculates times of cusps in the [0; 1] range and sets
    // `t_cusp1` and `t_cusp2` to them.
    void calculate_cusp_ts()
    {
        double t1, t2;

        // Find where the velocity vector is zero in the x dimension:
        std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_x, b_x, c_x, eps);

        // Find where V_y = 0:
        if (t_cusp1 == INFINITY && t_cusp2 == INFINITY)
        {
            std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_y, b_y, c_y, eps);
        }
        else
        {
            std::tie(t1, t2) = calculate_dim_cusp_ts(a_y, b_y, c_y);
            combine_dim_solution_ts(t_cusp1, t_cusp2, t1, t2);
        }

        // Find where V_z = 0:
        if (t_cusp1 == INFINITY && t_cusp2 == INFINITY)
        {
            std::tie(t_cusp1, t_cusp2) = calculate_dim_cusp_ts(a_z, b_z, c_z, eps);
        }
        else
        {
            std::tie(t1, t2) = calculate_dim_cusp_ts(a_z, b_z, c_z);
            combine_dim_solution_ts(t_cusp1, t_cusp2, t1, t2);
        }

        // Only accept cusps in the [0; 1] range:
        filter_ts_to_0_1(t_cusp1, t_cusp2);
    }

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
    std::tuple<double, double> calculate_dim_cusp_ts(double a, double b, double c, double eps=1e-15)
    {
        // V = 3 * a * t^2 + 2 * b * t + c
        if (abs(a) < eps)
        {
            // V = 2 * b * t + c
            if (abs(b) < eps)
            {
                // V = c
                if (abs(c) < eps)
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

            return std::make_tuple(
                (-b - det) / (3 * a),
                det < abs ?
                    NAN : // solption 2 = solution 1. Report 1 solution.
                    (-b + det) / (3 * a)); // solution 2 != solution 1.
        }
    }

    // Calculates inflection points in the [0; 1] range and sets
    // `t_inflect1` and `t_inflect2` to them.
    void calculate_inflection_ts()
    {
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
    }

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
    //         above for dimensions `d` and `e`.
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

        if (abs(t_squared_coeff) < eps)
        {
            // t_squared_coeff = 0, this is a linear equation.
            if (abs(t_coeff) < eps)
            {
                // t_coeff is also = 0, this is const = 0 equation.
                if (abs(const_coeff) < eps)
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

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H
