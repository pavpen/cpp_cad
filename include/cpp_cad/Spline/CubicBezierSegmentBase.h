#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASE_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASE_H

#include "../algebra/vector_3.h"


namespace cpp_cad
{

// Operators for using tuples as vectors:
using vector_3::operator+;
using vector_3::operator-;
using vector_3::operator*;

template <class PointType>
class CubicBezierSegmentBase
{
    private:

    // p0 is the start point.
    // p1 and p2 are control points.
    // p2 is the end point.
    PointType p0_value, p1_value, p2_value, p3_value;

    public:

    CubicBezierSegmentBase(PointType &p0, PointType &p1, PointType &p2, PointType &p3)
    : p0_value(p0),
        p1_value(p1),
        p2_value(p2),
        p3_value(p3)
    {}

    const PointType &p0() const
    {
        return p0_value;
    }

    void p0(PointType value)
    {
        p0_value = value;
    }

    const PointType &p1() const
    {
        return p1_value;
    }

    void p1(PointType value)
    {
        p1_value = value;
    }

    const PointType &p2() const
    {
        return p2_value;
    }

    void p2(PointType value)
    {
        p2_value = value;
    }

    const PointType &p3() const
    {
        return p3_value;
    }

    void p3(PointType value)
    {
        p3_value = value;
    }

    // Evaluates the location along the Bézier spline at time t.
    //     The location is the start point at t = 0, and at the end point at
    // t = 1.
    PointType evaluate(double t) const
    {
        double one_minus_t = 1.0 - t;
        double k0 = one_minus_t * one_minus_t * one_minus_t; // (1 - t)^3
        double k1 = 3.0 * one_minus_t * one_minus_t * t; // 3 * (1 - t)^2 * t
        double k2 = 3.0 * one_minus_t * t * t; // 3 * (1 - t) * t^2
        double k3 = t * t * t; // t^3

        return PointType(
            k0 * p0.x() + k1 * p1.x() + k2 * p2.x() + k3 * p3.x(),
            k0 * p0.y() + k1 * p1.y() + k2 * p2.y() + k3 * p3.y(),
            k0 * p0.z() + k1 * p1.z() + k2 * p2.z() + k3 * p3.z()
        );
    }

    // Evaluates the first derivative at time t.
    PointType first_derivative(double t) const
    {
        double one_minus_t = 1 - t;
        double k0 = -3.0 * one_minus_t * one_minus_t;
        double k1 = 3.0 * (1.0 - 4.0 * t + 3.0 * t * t);
        double k2 = 3.0 * t * (2.0 - 3.0 * t);
        double k3 = 3.0 * t * t;

        return PointType(
            k0 * p0[0] + k1 * p1[0] + k2 * p2[0] + k3 * p3[0],
            k0 * p0[1] + k1 * p1[1] + k2 * p2[1] + k3 * p3[1],
            k0 * p0[2] + k1 * p1[2] + k2 * p2[2] + k3 * p3[2]
        );
    }

    // Returns a cubic Bézier segment that coincides with the piece of this
    // Bézier segment from `t` = `t_cut` to `t` = 1.
    //
    // The `t` of the new segment in terms of `t` of the parent segmen is
    // `t_2 = (t - t_cut) / (1 - t_cut)`, where `t` is the `t` of this
    // segment.
    CubicBezierSegmentBase<PointType> split_from_t(double t_cut) const
    {
        std::tuple<double, double, double>
            p0 = vector_3::point_to_tuple(p0_value()),
            p1 = vector_3::point_to_tuple(p1_value()),
            p2 = vector_3::point_to_tuple(p2_value()),
            p3 = vector_3::point_to_tuple(p3_value()),
            p0_prime = p0 + (p1 - p0) * t_cut,
            p1_prime = p1 + (p2 - p1) * t_cut,
            p2_prime = p2 + (p3 - p2) * t_cut,
            p0_double_prime = p0_prime + (p1_prime - p0_prime) * t_cut,
            p1_double_prime = p1_prime + (p2_prime - p1_prime) * t_cut,
            p0_triple_prime =
                p0_double_prime + (p1_double_prime - p0_double_prime) * t_cut;

        return CubicBezierSegmentBase<PointType>(
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
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_BASE_H
