#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_H


namespace cpp_cad
{

// Forward declaration:
template <class PointType>
class CubicBezierSegmentLinearizer;

template <class PointType>
class CubicBezierSegment
{
    private:

    // p0 is the start point.
    // p1 and p2 are control points.
    // p2 is the end point.
    PointType p0_value, p1_value, p2_value, p3_value;

    public:

    CubicBezierSegment(PointType &p0, PointType &p1, PointType &p2, PointType &p3)
    : p0_value(p0),
        p1_value(p1),
        p2_value(p2),
        p3_value(p3)
    {}

    PointType &p0()
    {
        return p0_value;
    }

    void p0(PointType value)
    {
        p0_value = value;
    }

    PointType &p1()
    {
        return p1_value;
    }

    void p1(PointType value)
    {
        p1_value = value;
    }

    PointType &p2()
    {
        return p2_value;
    }

    void p2(PointType value)
    {
        p2_value = value;
    }

    PointType &p3()
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
    PointType evaluate(double t)
    {
        double one_minus_t = 1.0 - t;
        double k0 = one_minus_t * one_minus_t * one_minus_t; // (1 - t)^3
        double k1 = 3.0 * one_minus_t * one_minus_t * t; // 3 * (1 - t)^2 * t
        double k2 = 3.0 * one_minus_t * t * t; // 3 * (1 - t) * t^2
        double k3 = t * t * t; // t^3

        return PointType(
            k0 * p0[0] + k1 * p1[0] + k2 * p2[0] + k3 * p3[0],
            k0 * p0[1] + k1 * p1[1] + k2 * p2[1] + k3 * p3[1],
            k0 * p0[2] + k1 * p1[2] + k2 * p2[2] + k3 * p3[2]
        );
    }

    // Evaluates the first derivative at time t.
    PointType first_derivative(double t)
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

    std::vector<double> linearize_ts(
        double max_lateral_distance, double eps=1e-15)
    {
        CubicBezierSegmentLinearizer<PointType> linearizer(*this, eps);

        return linearizer.linearize_ts(max_lateral_distance);
    }
};

}


#include "CubicBezierSegmentLinearizer.h"


#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_H
