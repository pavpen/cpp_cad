#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include "../algebra/vector_3.h"
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

    // `max_lateral_distance` = The `f` parameter in the Hain, Ahmad, Langan
    //         paper.  I.e. an estimate of the maximum (lateral) distance of a
    //         line segment from the curve.
    double max_lateral_distance;

    // The Bézier basis matrix.  If multiplied by [t^3, t^2, t, 1], it gives
    // the location of the point on the Bézier curve at parameter t.
    double a_x, b_x, c_x, d_x;
    double a_y, b_y, c_y, d_y;
    double a_z, b_z, c_z, d_z;

    // Values of the `t` parameter at which inflection points occur.  (I.e.,
    // when the velocity and the acceleration vectors become collinear.)
    double t_inflect1, t_inflect2;

    // Value (`t_f` in Hain, Ahmad, and Langan) used when estimating the error
    // around inflection points.
    double t_f;

    // Values of `t` at which line segments for each inflection point start
    // and end.
    double t_inflect1_start, t_inflect1_end;
    double t_inflect2_start, t_inflect2_end;

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

    // The orthonormal basis vectors in the coordinate system that travels
    // along the curve segment.  The `q` axis (`r` in Hain, Ahmad, Langan) is
    // in the direction from the first control point to the second.  The
    // `r` axis (`s` in Hain, Ahmad, Langan) is orthogonal to `q` and in the
    // plane of the first 3 control points.  The `s` axis forms a right-handed
    // coordinate system with `q` and `r`.
    double q_x, q_y, q_z;
    double r_x, r_y, r_z;
    double s_x, s_y, s_z;

    public:

    CubicBezierSegmentLinearizer(
        const CubicBezierSegment<PointType> &segment,
        double max_lateral_distance, double eps = 1e-15)
    : segment(segment),
        max_lateral_distance(max_lateral_distance),
        eps(eps)
    {}

    // Returns the values of the `t` parameter for the points on the curve
    // that can be connected by line segments.
    std::vector<double> linearize_ts()
    {
        calculate_t_basis_matix();
        calculate_cusp_ts();
        calculate_inflection_ts();
        calculate_inflect_line_segments();
        calculate_qrs_basis();

        // Split the curve at cusp points, use line segments around inflection
        // points, and subdivide the rest using parabolic error approximation.
    }

    private:

    // Requires inflection line segments, `q`-`r`-`s` basis, and `t_f` to have
    // been calculated.  Only works in the [0; 1] range.
    template <class OutputIterator>
    inline OutputIterator &linearize_cusp_free_range(double t_start, double t_end,
        OutputIterator &output_iter)
    {
        // We have to perform parabolic linerization only on ranges that don't
        // intersect inflection point line segment ranges.
        linearize_cusp_free_range_to_inflection(
            t_start, t_end, t_inflect1_start, t_inflect1_end, output_iter);

        if (std::isnan(t_start))
        {
            return;
        }

        linearize_cusp_free_range_to_inflection(
            t_start, t_end, t_inflect2_start, t_inflect2_end, output_iter);

        if (std::isnan(t_start))
        {
            return;
        }

        return parabolic_linearize_range(t_start, t_end, output_iter);
    }

    template <class OutputIterator>
    inline void linearize_cusp_free_range_to_inflection(
        double &t_start, double &t_end, double t_inflect_start, double t_inflect_end,
        OutputIterator &output_iter)
    {
        if (std::isnan(t_inflect_start) || t_end <= t_inflect_start + eps)
        {
            return output_iter;
        }

        if (t_start + eps < t_inflect_end)
        {
            // Inflection point range intersects [t_start; t_end].
            if (t_start + eps < t_inflect_start)
            {
                // Linearize to inflection range start:
                output_iter = parabolic_linearize_range(
                    t_start, t_inflect_start, output_iter);
            }
            if (t_end <= t_inflect_end + eps)
            {
                // t_end is in the inflection range:
                (*output_iter) = t_end;
                ++output_iter;

                t_start = NAN;

                return output_iter;
            }
            // Line segment to the end of inflection range 1:
            (*output_iter) = t_inflect_end;
            ++output_iter;
            // Continue from t_inflect_end:
            t_start = t_inflect_end;
        }
    }

    template <class OutputIterator>
    inline OutputIterator &parabolic_linearize_range(double t_start, double t_end,
        OutputIterator &output_iter)
    {
        if (t_end - t_start < eps)
        {
            return output_iter;
        }

        double t;

        if (t_start <= eps)
        {
            t = parabolic_split_t();
        }
        if (t + eps >= t_end)
        {
            (*output_iter) = t_end;
            ++output_iter;

            return output_iter;
        }
        (*output_iter) = t;
        ++output_iter;
        t_start = t;

        CubicBezierSegmentLinearizer
            linearizer(segment, max_lateral_distance, eps);

        while (true)
        {
            CubicBezierSegment sub_segment =
                split_end_segment_from_t(t_start);

            linearizer.segment = sub_segment;
            linearizer.calculate_qrs_basis();
            t = linearizer.parabolic_split_t();

            if (t + eps >= t_end)
            {
                (*output_iter) = t_end;
                ++output_iter;

                return output_iter;
            }
            (*output_iter) = t;
            ++output_iter;
            t_start = t;
        }
    }

    // Returns a cubic Bézier segment that coincides with the piece of this
    // Bézier segment from `t` = `t_cut` to `t` = 1.
    //
    // The `t` of the new segment is `t_2 = (t - t_cut) / (1 - t_cut)`,
    // where `t` is the `t` of this segment.
    CubicBezierSegment split_end_segment_from_t(double t_cut)
    {
        std::tuple<double, double, double>
            p0 = vector_3::point_to_tuple(segment.p0()),
            p1 = vector_3::point_to_tuple(segment.p1()),
            p2 = vector_3::point_to_tuple(segment.p2()),
            p3 = vector_3::point_to_tuple(segment.p3()),
            p0_prime = p0 + (p1 - p0) * t_cut,
            p1_prime = p1 + (p2 - p1) * t_cut,
            p2_prime = p2 + (p3 - p2) * t_cut,
            p0_double_prime = p0_prime + (p1_prime - p0_prime) * t_cut,
            p1_double_prime = p1_prime + (p2_prime - p1_prime) * t_cut,
            p0_triple_prime =
                p0_double_prime + (p1_double_prime - p0_double_prime) * t_cut;

        return CubicBezierSegment(
            PointType(
                std::get<0>(p0_triple_prime),
                std::get<1>(p0_triple_prime),
                std::get<2>(p0_triple_prime) ),
            PointType(
                std::get<0>(p1_double_prime),
                std::get<1>(p1_double_prime),
                std::get<2>(p1_double_prime) ),
            PointType(
                std::get<0>(p2_prime),
                std::get<1>(p2_prime),
                std::get<2>(p2_prime) ),
            PointType(
                std::get<0>(p3),
                std::get<1>(p3),
                std::get<2>(p3) )
        );
    }

    // Return the `t` parameter at which to split the curve from the
    // beginning, so that it's distance from a line segment is no more than
    // about `max_lateral_distance`.
    //
    // Requires the `q`, `r`, `s` basis to have been calculated.
    inline double parabolic_split_t()
    {
        double r2 = vector_3::dot_3(
            segment.p2().x(), segment.p2().y(), segment.p2().z(),
            r_x, r_y, r_z);
        double s2 = vector_3::dot_3(
            segment.p2().x(), segment.p2().y(), segment.p2().z(),
            s_x, s_y, s_z);

        return 2 * sqrt( max_lateral_distance / 3 * sqrt(r2 * r2 + s2 * s2) );
    }

    inline void calculate_inflect_line_segments()
    {
        if (std::isnan(t_inflect1))
        {
            if (std::isnan(t_inflect2))
            {
                t_inflect1_start = t_inflect1_end = NAN;
            }
            else
            {
                std::tie(t_inflect1_start, t_inflect1_end) =
                    inflection_split_t_range(t_inflect2);
            }
            t_inflect2_start = t_inflect2_end = NAN;
        }
        else
        {
            std::tie(t_inflect1_start, t_inflect1_end) =
                inflection_split_t_range(t_inflect1);
            if (std::isnan(t_inflect2))
            {
                t_inflect2_start = t_inflect2_end = NAN;
                return;
            }
            if (std::isnan(t_inflect1_start))
            {
                std::tie(t_inflect1_start, t_inflect1_end)
                    = inflection_split_t_range(t_inflect2);
                t_inflect2_start  = t_inflect2_end = NAN;
                return;
            }
            if (!std::isnan(t_inflect2))
            {
                std::tie(t_inflect2_start, t_inflect2_end) =
                    inflection_split_t_range(t_inflect2);
                if (std::isnan(t_inflect2_start))
                {
                    return;
                }
                if (t_inflect2_start < t_inflect1_end)
                {
                    // The two ranges overlap.
                    if (t_inflect2_start <= t_inflect1_start + eps)
                    {
                        // Range 2 includes range 1.
                        t_inflect1_start = t_inflect2_start;
                        t_inflect1_end = t_inflect2_end;
                        t_inflect1_start = NAN;
                        t_inflect1_end = NAN;
                        return;
                    }
                    if (t_inflect1_end + eps >= t_inflect2_end)
                    {
                        // Range 1 includes range 2.
                        t_inflect2_start = NAN;
                        t_inflect2_end = NAN;
                        return;
                    }
                    t_inflect1_end = t_inflect2_start =
                        (t_inflect1_end + t_inflect2_start) / 2;
                }
            }
        }
    }

    // Requires `t_f` to have been calculated.
    inline std::tuple<double, double> inflection_split_t_range(
        double t_inflection)
    {
        double t_start = t_inflection - t_f * (1 - t_inflection);
        double t_end   = t_inflection + t_f * (1 - t_inflection);

        if (t_start < 0)
        {
            t_start = 0;
        }
        else if (t_start >= 1)
        {
            return std::make_tuple(NAN, NAN);
        }

        if (t_end <= 0)
        {
            return std::make_tuple(NAN, NAN);
        }
        else if (t_end > 1)
        {
            t_end = 1;
        }

        return std::make_tuple(t_start, t_end);
    }

    // Requires the `q`, `r`, `s` basis to have been calculated.
    inline void calculate_t_f()
    {
        double r3 = dot_3(
            segment.p3().x(), segment.p3().y(), segment.p3().z(),
            r_x, r_y, r_z);
        double s3 = dot_3(
            segment.p3().x(), segment.p3().y(), segment.p3().z(),
            s_x, s_y, s_z);

        t_f = cbrt(max_lateral_distance / sqrt(r3 * r3 + s3 * s3));
    }

    void calculate_qrs_basis()
    {
        if (distance_3_squared(p0, p1) < eps)
        {
            calculate_qrs_basis(p0, p2, p3);
        }
        else if (distance_3_squared(p0, p2) < eps)
        {
            calculate_qrs_basis(p0, p1, p3);
        }
        else if (distance_3_squared(p0, p3) < eps)
        {
            calculate_qrs_basis(p0, p1, p2);
        }
        else if (distance_3_squared(p1, p2) < eps)
        {
            calculate_qrs_basis(p0, p1, p3);
        }
        else
        {
            calculate_qrs_basis(p0, p1, p2);
        }
    }

    // Construct `q`, `r` and `s` basis vectors from 3 points,
    // `p0`, `p1` and `p2`.
    inline void calculate_qrs_basis(
        const PointType &p0, const PointType &p1, const PointType &p2)
    {
        if (distance_3_squared(p0, p1) < eps)
        {
            calculate_qrs_basis(p0, p2);
            return;
        }
        else if (distance_3_squared(p0, p2) < eps ||
            distance_3_squared(p1, p2) < eps)
        {
            calculate_qrs_basis(p0, p1);
            return;
        }

        q_x = p1.x() - p0.x();
        q_y = p1.y() - p0.y();
        q_z = p1.z() - p0.z();

        double norm = sqrt(norm_3_squared(q_x, q_y, q_z));

        q_x /= norm;
        q_y /= norm;
        q_z /= norm;

        r_x = p2().x - p0.x();
        r_y = p2().y - p0.y();
        r_z = p2().z - p0.z();
        norm = sqrt(norm_3_squared(r_x, r_y, r_z));
        r_x /= norm;
        r_y /= norm;
        r_z /= norm;

        std::tie(s_x, s_y, s_z) = cross_3(q_x, q_y, q_z, r_x, r_y, r_z);
        std::tie(r_x, r_y, r_z) = cross_3(s_x, s_y, s_z, q_x, q_y, q_z);
    }

    // Construct `q`, `r` and `s` basis vectors from only 2 points,
    // `p0`, and `p1`.
    inline void calculate_qrs_basis(const PointType &p0, const PointType &p1)
    {
        if (distance_3_squared(p0, p1) < eps)
        {
            calculate_qrs_basis(p0);
            return;
        }

        q_x = p1.x() - p0.x();
        q_y = p1.y() - p0.y();
        q_z = p1.z() - p0.z();

        double norm = sqrt(norm_3_squared(q_x, q_y, q_z));

        q_x /= norm;
        q_y /= norm;
        q_z /= norm;

        std::tie(r_x, r_y, r_z) = find_perpendicular_axis(q_x, q_y, q_z);

        std::tie(s_x, s_y, s_z) = cross_3(q_x, q_y, q_z, r_x, r_y, r_z);
        std::tie(r_x, r_y, r_z) = cross_3(s_x, s_y, s_z, q_x, q_y, q_z);
    }

    // Construct some `q`, `r` and `s` basis vectors when all control points
    // concide.
    inline void calculate_qrs_basis(const PointType &)
    {
        // All points coincide.  Use x, y, z coordinates:
        q_x = 1; q_y = 0; q_z = 0;
        r_x = 0; r_y = 1; r_z = 0;
        s_x = 0; s_y = 0; s_z = 1;
    }

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
    //
    // Requires the Bézier basis matrix to have been calculated.
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
