#ifndef _CPP_CAD_TRANSFORM_ITERATOR_H
#define _CPP_CAD_TRANSFORM_ITERATOR_H

#include "Circle_TransformIterator.h"
#include "Rotate_TransformIterator.h"


namespace cpp_cad
{
    class TransformIterator
    {
        public:

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

        template <Aff_transformation_3 TransformFunctor(double angle)>
        class Rotation
        {
            private:

            double start_angle;
            double end_angle;
            int subdivision_c;
            bool closed;

            public:

            Rotation<TransformFunctor>(
                double start_angle = 0, double end_angle = 2 * M_PI,
                int subdivision_c = 16, bool closed = true)
            : start_angle(start_angle),
                end_angle(end_angle),
                subdivision_c(subdivision_c),
                closed(closed)
            {}

            RotateTransformIterator<TransformFunctor> begin()
            {
                return RotateTransformIterator<TransformFunctor>(
                    start_angle, end_angle, subdivision_c, 0,
                    closed ? subdivision_c : subdivision_c + 1);
            }

            RotateTransformIterator<TransformFunctor> end()
            {
                return RotateTransformIterator<TransformFunctor>(
                    start_angle, end_angle, subdivision_c,
                    closed ? subdivision_c : subdivision_c + 1);
            }
        };

        typedef Rotation<Aff_transformation_3::rotate_x> XRotation;
        typedef Rotation<Aff_transformation_3::rotate_y> YRotation;
        typedef Rotation<Aff_transformation_3::rotate_z> ZRotation;
    };
}

#endif // _CPP_CAD_TRANSFORM_ITERATOR_H
