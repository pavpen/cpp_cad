#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

#include "../algebra/vector_3.h"
#include "CubicBezierSegmentCuspCalculator.h"
#include "CubicBezierSegmentInflectionCalculator.h"
#include "CubicBezierSegmentQrsBasisCalculator.h"


namespace cpp_cad
{

template <class PointType>
class CubicBezierSegmentLinearizer;

// Uses the algorithm described by Hain, Ahmad, and Langan in
// <https://pdfs.semanticscholar.org/8963/c06a92d6ca8868348b0930bbb800ff6e7920.pdf>
// to split a cubic Bézier curve segment into linear segments.
template <class PointType>
class CubicBezierSegmentLinearizer
: virtual protected CubicBezierSegmentCuspCalculator<PointType>,
    virtual protected CubicBezierSegmentInflectionCalculator<PointType>,
    virtual protected CubicBezierSegmentQrsBasisCalculator<PointType>
{
    public:

    // C++ compilers need a lot of help:
    using CalculatorBase = CubicBezierSegmentCalculatorBase<PointType>;
    using BezierSegment = typename CalculatorBase::BezierSegment;
    using BasisMatrixCalculator =
        CubicBezierSegmentBasisMatrixCalculator<PointType>;
    using CuspCalculator = CubicBezierSegmentCuspCalculator<PointType>;
    using InflectionCalculator =
        CubicBezierSegmentInflectionCalculator<PointType>;
    using QrsBasisCalculator =
        CubicBezierSegmentQrsBasisCalculator<PointType>;

    using CalculatorBase::segment;
    using CalculatorBase::eps;
    using QrsBasisCalculator::r_x;
    using QrsBasisCalculator::r_y;
    using QrsBasisCalculator::r_z;
    using QrsBasisCalculator::s_x;
    using QrsBasisCalculator::s_y;
    using QrsBasisCalculator::s_z;
    using BasisMatrixCalculator::calculate_t_basis_matrix;
    using InflectionCalculator::calculate_inflection_ts;
    using QrsBasisCalculator::calculate_qrs_basis;

    private:

    // `max_lateral_distance` = The `f` parameter in the Hain, Ahmad, Langan
    //         paper.  I.e. an estimate of the maximum (lateral) distance of a
    //         line segment from the curve.
    double max_lateral_distance;

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
        const BezierSegment &segment,
        double max_lateral_distance, double eps = 1e-15)
    : CalculatorBase(segment, eps),
        BasisMatrixCalculator(segment, eps),
        CuspCalculator(segment, eps),
        InflectionCalculator(segment, eps),
        QrsBasisCalculator(segment, eps),
        max_lateral_distance(max_lateral_distance)
    {}

    // Returns the values of the `t` parameter for the points on the curve
    // that can be connected by line segments.
    template <class OutputIterator>
    OutputIterator &linearize_ts(OutputIterator &output_iter)
    {
        calculate_t_basis_matrix();
        std::tie(t_cusp1, t_cusp2) = this->calculate_cusp_ts();
        std::tie(t_inflect1, t_inflect2) = calculate_inflection_ts();
        calculate_qrs_basis();
        calculate_t_f();
        calculate_inflect_line_segments();

        // Split the curve at cusp points, use line segments around inflection
        // points, and subdivide the rest using parabolic error approximation.
        if (std::isnan(t_cusp1) || t_cusp1 == INFINITY)
        {
            // This curve segment has no cusps, or is a one-point segment
            // where every point is a cusp.
            return linearize_cusp_free_range(output_iter, 0, 1);
        }

        double t_start;

        if (t_cusp1 > eps)
        {
            linearize_cusp_free_range(output_iter, 0, t_cusp1 - eps);
            t_start = t_cusp1 + eps;
        }
        else
        {
            t_start = t_cusp1 + eps;
        }

        if (std::isnan(t_cusp2) || t_cusp2 + eps >= 1)
        {
            return linearize_cusp_free_range(output_iter, t_start, 1);
        }
        else
        {
            // Merge cusps that are too close together:
            if (t_cusp2 - eps > t_start + eps)
            {
                linearize_cusp_free_range(output_iter, t_start, t_cusp2 - eps);
            }
            return linearize_cusp_free_range(output_iter, t_cusp2 + eps, 1);
        }
    }

    private:

    // Requires inflection line segments, `q`-`r`-`s` basis, and `t_f` to have
    // been calculated.  Only works in the [0; 1] range.
    template <class OutputIterator>
    inline OutputIterator &linearize_cusp_free_range(
        OutputIterator &output_iter, double t_start, double t_end)
    {
        // We have to perform parabolic linerization only on ranges that don't
        // intersect inflection point line segment ranges.
        linearize_cusp_free_range_to_inflection(
            output_iter, t_start, t_end, t_inflect1_start, t_inflect1_end);

        if (std::isnan(t_start))
        {
            return output_iter;
        }

        linearize_cusp_free_range_to_inflection(
            output_iter, t_start, t_end, t_inflect2_start, t_inflect2_end);

        if (std::isnan(t_start))
        {
            return output_iter;
        }

        return parabolic_linearize_range(output_iter, t_start, t_end);
    }

    template <class OutputIterator>
    inline OutputIterator &linearize_cusp_free_range_to_inflection(
        OutputIterator &output_iter,
        double &t_start, double &t_end, double t_inflect_start, double t_inflect_end)
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
                    output_iter, t_start, t_inflect_start);
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

    // Adds line segments to `output_iter` for the curve segment from
    // `t_start` to `t_end`.
    //     Uses parabolic estimation of the error, so the curve segment must
    // have no cusps or inflection points.
    //     Requires the `q`-`r`-`s` basis to have been calculated.
    template <class OutputIterator>
    inline OutputIterator &parabolic_linearize_range(
        OutputIterator &output_iter, double t_start, double t_end)
    {
        if (t_end - t_start < eps)
        {
            return output_iter;
        }

        double t;

        if (t_start <= eps)
        {
            t = parabolic_split_t();
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

        while (true)
        {
            CubicBezierSegmentLinearizer
                linearizer(
                    segment.split_from_t(t_start), max_lateral_distance, eps);

            linearizer.calculate_qrs_basis();
            t = linearizer.parabolic_split_t();
            t = t_start + (1 - t_start) * t;

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

    // Return the `t` parameter at which to split the curve from the
    // beginning, so that it's distance from a line segment is no more than
    // about `max_lateral_distance`.
    //
    // Requires the `q`, `r`, `s` basis to have been calculated.
    inline double parabolic_split_t()
    {
        std::tuple<double, double, double>
            p2 = vector_3::point_to_tuple(segment.p2());
        double r2 = vector_3::dot_3(p2, std::make_tuple(r_x, r_y, r_z));
        double s2 = vector_3::dot_3(p2, std::make_tuple(s_x, s_y, s_z));

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
        std::tuple<double, double, double> p3 =
            vector_3::point_to_tuple(segment.p3());
        double r3 = vector_3::dot_3(p3, std::make_tuple(r_x, r_y, r_z));
        double s3 = vector_3::dot_3(p3, std::make_tuple(s_x, s_y, s_z));

        t_f = cbrt(max_lateral_distance / sqrt(r3 * r3 + s3 * s3));
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_LINEARIZER_H
