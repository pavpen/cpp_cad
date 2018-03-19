#ifndef _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H

#include "../Aff_transformation_3.h"
#include "../reference_frame.h"


namespace cpp_cad
{
    template <Aff_transformation_3 TransformFunctor(double angle)>
    class RotateTransformIterator
    {
        private:

        int subdivision_c;
        double angle;
        double angle_step;
        int subdivision_i;

        public:

        typedef std::bidirectional_iterator_tag iterator_category;

        RotateTransformIterator(
            double start_angle = 0, double end_angle = 2 * M_PI,
            int subdivision_c = 16, int subdivision_i = 0)
        : subdivision_c(subdivision_c),
            angle(start_angle),
            angle_step((end_angle - start_angle) / subdivision_c),
            subdivision_i(subdivision_i)
        {}

        RotateTransformIterator(const RotateTransformIterator &source)
        : subdivision_c(source.subdivision_c),
            angle(source.angle),
            angle_step(source.angle_step),
            subdivision_i(source.subdivision_i)
        {}

        RotateTransformIterator &operator++()
        {
            ++subdivision_i;
            angle += angle_step;

            return *this;
        }

        RotateTransformIterator &operator--()
        {
            --subdivision_i;
            angle -= angle_step;

            return *this;
        }

        RotateTransformIterator operator++(int)
        {
            RotateTransformIterator res(*this);

            ++subdivision_i;
            angle += angle_step;

            return res;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator==(const RotateTransformIterator &rhs) const
        {
            return subdivision_i == rhs.subdivision_i;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator!=(const RotateTransformIterator &rhs) const
        {
            return subdivision_i != rhs.subdivision_i;
        }

        Aff_transformation_3 operator*() const
        {
            return TransformFunctor(angle);
        }

        int steps_left() const
        {
            return subdivision_c - subdivision_i;
        }
    };

    typedef RotateTransformIterator<Aff_transformation_3::rotate_x>   XRotateTransformIterator;
    typedef RotateTransformIterator<Aff_transformation_3::rotate_y>   YRotateTransformIterator;
    typedef RotateTransformIterator<Aff_transformation_3::rotate_z>   ZRotateTransformIterator;
}

#endif // _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H
