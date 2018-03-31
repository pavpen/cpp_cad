#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_QRS_BASIS_CALCULATOR_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_QRS_BASIS_CALCULATOR_H

#include "../algebra/vector_3.h"
#include "CubicBezierSegmentCalculatorBase.h"


namespace cpp_cad
{

// Calculates the orthonormal basis vectors of a coordinate system along the
// control points of a cubic Bézier curve segment, used for estimating errors
// when converting a curve into line segments by Hain, Ahmad, and Langan in
// <https://pdfs.semanticscholar.org/8963/c06a92d6ca8868348b0930bbb800ff6e7920.pdf>.
template <class PointType>
class CubicBezierSegmentQrsBasisCalculator
: virtual protected CubicBezierSegmentCalculatorBase<PointType>
{
    public:

    using CalculatorBase = CubicBezierSegmentCalculatorBase<PointType>;
    using BezierSegment = typename CalculatorBase::BezierSegment;

    using CalculatorBase::eps;
    using CalculatorBase::p0;
    using CalculatorBase::p1;
    using CalculatorBase::p2;
    using CalculatorBase::p3;

    protected:

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

    CubicBezierSegmentQrsBasisCalculator(
        const BezierSegment &segment, double eps = 1e-15)
    : CubicBezierSegmentCalculatorBase<PointType>(segment, eps)
    {}

    void calculate_qrs_basis()
    {
        if (distance_3_squared(p0(), p1()) < eps)
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

    protected:

    // Construct `q`, `r` and `s` basis vectors from 3 points,
    // `p0`, `p1` and `p2`.
    inline void calculate_qrs_basis(
        const PointType &p0, const PointType &p1, const PointType &p2)
    {
        if (vector_3::distance_3_squared(p0, p1) < eps)
        {
            calculate_qrs_basis(p0, p2);
            return;
        }
        else if (vector_3::distance_3_squared(p0, p2) < eps ||
            vector_3::distance_3_squared(p1, p2) < eps)
        {
            calculate_qrs_basis(p0, p1);
            return;
        }

        q_x = p1.x() - p0.x();
        q_y = p1.y() - p0.y();
        q_z = p1.z() - p0.z();

        double norm = sqrt(vector_3::norm_3_squared(q_x, q_y, q_z));

        q_x /= norm;
        q_y /= norm;
        q_z /= norm;

        r_x = p2().x - p0.x();
        r_y = p2().y - p0.y();
        r_z = p2().z - p0.z();
        norm = sqrt(vector_3::norm_3_squared(r_x, r_y, r_z));
        r_x /= norm;
        r_y /= norm;
        r_z /= norm;

        std::tie(s_x, s_y, s_z) =
            vector_3::cross_3(q_x, q_y, q_z, r_x, r_y, r_z);
        std::tie(r_x, r_y, r_z) =
            vector_3::cross_3(s_x, s_y, s_z, q_x, q_y, q_z);
    }

    // Construct `q`, `r` and `s` basis vectors from only 2 points,
    // `p0`, and `p1`.
    inline void calculate_qrs_basis(const PointType &p0, const PointType &p1)
    {
        if (vector_3::distance_3_squared(p0, p1) < eps)
        {
            calculate_qrs_basis(p0);
            return;
        }

        q_x = p1.x() - p0.x();
        q_y = p1.y() - p0.y();
        q_z = p1.z() - p0.z();

        double norm = sqrt(vector_3::norm_3_squared(q_x, q_y, q_z));

        q_x /= norm;
        q_y /= norm;
        q_z /= norm;

        std::tie(r_x, r_y, r_z) =
            vector_3::find_perpendicular_axis(q_x, q_y, q_z);

        std::tie(s_x, s_y, s_z) =
            vector_3::cross_3(q_x, q_y, q_z, r_x, r_y, r_z);
        std::tie(r_x, r_y, r_z) =
            vector_3::cross_3(s_x, s_y, s_z, q_x, q_y, q_z);
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
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_QRS_BASIS_CALCULATOR_H
