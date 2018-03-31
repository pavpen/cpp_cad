#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_H

#include "CubicBezierSegmentBase.h"
#include "CubicBezierSegmentCuspCalculator.h"
#include "CubicBezierSegmentInflectionCalculator.h"
#include "CubicBezierSegmentLinearizer.h"

namespace cpp_cad
{

template <class PointType>
class CubicBezierSegment : public CubicBezierSegmentBase<PointType>
{
    public:

    CubicBezierSegment(PointType &p0, PointType &p1, PointType &p2, PointType &p3)
    : CubicBezierSegmentBase<PointType>(p0, p1, p2, p3)
    {}

    template <class OutputIterator>
    OutputIterator &linearize_ts(
        OutputIterator &output_iter,
        double max_lateral_distance, double eps=1e-15)
    {
        CubicBezierSegmentLinearizer<PointType> linearizer(*this, eps);

        return linearizer.linearize_ts(output_iter, max_lateral_distance);
    }

    std::tuple<double, double> calculate_cusp_ts(double eps=1e-15)
    {
        CubicBezierSegmentCuspCalculator<PointType> calculator(*this, eps);

        calculator.calculate_t_basis_matrix();

        return calculator.calculate_cusp_ts();
    }

    std::tuple<double, double> calculate_inflection_ts(double eps=1e-15)
    {
        CubicBezierSegmentInflectionCalculator<PointType>
            calculator(*this, eps);

        calculator.calculate_t_basis_matrix();

        return calculator.calculate_inflection_ts();
    }
};

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_H
