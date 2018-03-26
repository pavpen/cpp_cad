#ifndef _CPP_CAD_TRANSFORM_ITERATOR_CIRCLE_H
#define _CPP_CAD_TRANSFORM_ITERATOR_CIRCLE_H

#include "../Aff_transformation_3.h"
#include "CircleTransformIterator.h"


namespace cpp_cad
{
    namespace TransformIterator
    {
        template <Aff_transformation_3 TransformFunctor(double angle, double r)>
        class Circle
        {
            private:

            double r;
            double start_angle;
            double end_angle;
            int subdivision_c;
            bool closed;

            public:

            Circle(
                double r, double start_angle = 0, double end_angle = 2 * M_PI,
                int subdivision_c = 16, bool closed = true)
            : r(r),
                start_angle(start_angle),
                end_angle(end_angle),
                subdivision_c(subdivision_c),
                closed(closed)
            {}

            CircleTransformIterator<TransformFunctor> begin()
            {
                return CircleTransformIterator<TransformFunctor>(
                    r, start_angle, end_angle, subdivision_c, 0,
                    closed ? subdivision_c : subdivision_c + 1);
            }

            CircleTransformIterator<TransformFunctor> end()
            {
                return CircleTransformIterator<TransformFunctor>(
                    r, start_angle, end_angle, subdivision_c,
                    closed ? subdivision_c : subdivision_c + 1);
            }
        };
    }
}

#endif // _CPP_CAD_TRANSFORM_ITERATOR_CIRCLE_H