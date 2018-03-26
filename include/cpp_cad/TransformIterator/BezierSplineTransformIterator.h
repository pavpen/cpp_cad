#ifndef _CPP_CAD_BEZIER_SPLINE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_BEZIER_SPLINE_TRANSFORM_ITERATOR_H

#include <iterator>

#include "../Aff_transformation_3.h"


namespace
{

    class BezierSplineTransformIterator
    {
        private:

        double angle;
        double angle_step;
        int subdivision_i;
        int subdivision_i_end;

        public:

        typedef std::bidirectional_iterator_tag iterator_category;

        BezierSplineTransformIterator(
            double start_angle = 0, double end_angle = 2 * M_PI,
            int subdivision_c = 16, int subdivision_i = 0,
            int subdivision_i_end = -1)
        : angle(start_angle),
            angle_step((end_angle - start_angle) / subdivision_c),
            subdivision_i(subdivision_i),
            subdivision_i_end(subdivision_i_end >= 0 ? subdivision_i_end : subdivision_c)
        {}

        BezierSplineTransformIterator(const BezierSplineTransformIterator &source)
        : angle(source.angle),
            angle_step(source.angle_step),
            subdivision_i(source.subdivision_i),
            subdivision_i_end(source.subdivision_i_end)
        {}

        BezierSplineTransformIterator &operator++()
        {
            ++subdivision_i;
            angle += angle_step;

            return *this;
        }

        BezierSplineTransformIterator &operator--()
        {
            --subdivision_i;
            angle -= angle_step;

            return *this;
        }

        BezierSplineTransformIterator operator++(int)
        {
            BezierSplineTransformIterator res(*this);

            ++subdivision_i;
            angle += angle_step;

            return res;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator==(const BezierSplineTransformIterator &rhs) const
        {
            return subdivision_i == rhs.subdivision_i;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator!=(const BezierSplineTransformIterator &rhs) const
        {
            return subdivision_i != rhs.subdivision_i;
        }

        Aff_transformation_3 operator*() const
        {
            return TransformFunctor(angle);
        }

        int steps_left() const
        {
            return subdivision_i_end - subdivision_i;
        }
    };

}

#endif // _CPP_CAD_BEZIER_SPLINE_TRANSFORM_ITERATOR_H
