#ifndef _CPP_CAD_TRANSFORM_ITERATOR_ROTATION_H
#define _CPP_CAD_TRANSFORM_ITERATOR_ROTATION_H

#include "../Aff_transformation_3.h"
#include "RotateTransformIterator.h"


namespace cpp_cad
{
    namespace TransformIterator
    {
        template <Aff_transformation_3 TransformFunctor(double angle)>
        class Rotation
        {
            private:

            double start_angle;
            double end_angle;
            int subdivision_c;
            bool closed;

            public:

            typedef RotateTransformIterator<TransformFunctor> TransformIterator;

            Rotation<TransformFunctor>(
                double start_angle = 0, double end_angle = 2 * M_PI,
                int subdivision_c = 16, bool closed = true)
            : start_angle(start_angle),
                end_angle(end_angle),
                subdivision_c(subdivision_c),
                closed(closed)
            {}

            TransformIterator begin()
            {
                return RotateTransformIterator<TransformFunctor>(
                    start_angle, end_angle, subdivision_c, 0,
                    closed ? subdivision_c : subdivision_c + 1);
            }

            TransformIterator end()
            {
                return RotateTransformIterator<TransformFunctor>(
                    start_angle, end_angle, subdivision_c,
                    closed ? subdivision_c : subdivision_c + 1);
            }
        };

        typedef Rotation<Aff_transformation_3::rotate_x> XRotation;
        typedef Rotation<Aff_transformation_3::rotate_y> YRotation;
        typedef Rotation<Aff_transformation_3::rotate_z> ZRotation;
    }
}

#endif // _CPP_CAD_TRANSFORM_ITERATOR_ROTATION_H
