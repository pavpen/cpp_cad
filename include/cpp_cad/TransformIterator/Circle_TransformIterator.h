#ifndef _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H

#include "../Aff_transformation_3.h"
#include "../reference_frame.h"


namespace cpp_cad
{
    template <Aff_transformation_3 TransformFunctor(double angle, double r)>
    class CircleTransformIterator
    {
        private:

        double r;
        double angle;
        double angle_step;
        int subdivision_i;
        int subdivision_i_end;

        public:

        typedef std::bidirectional_iterator_tag iterator_category;

        CircleTransformIterator(
            double r, double start_angle = 0, double end_angle = 2 * M_PI,
            int subdivision_c = 16, int subdivision_i = 0,
            int subdivision_i_end = -1)
        : r(r),
            angle(start_angle),
            angle_step((end_angle - start_angle) / subdivision_c),
            subdivision_i(subdivision_i),
            subdivision_i_end(subdivision_i_end >= 0 ? subdivision_i_end : subdivision_c)
        {}

        CircleTransformIterator(const CircleTransformIterator &source)
        : r(source.r),
            angle(source.angle),
            angle_step(source.angle_step),
            subdivision_i(source.subdivision_i),
            subdivision_i_end(source.subdivision_i_end)
        {}

        CircleTransformIterator &operator++()
        {
            ++subdivision_i;
            angle += angle_step;

            return *this;
        }

        CircleTransformIterator &operator--()
        {
            --subdivision_i;
            angle -= angle_step;

            return *this;
        }

        CircleTransformIterator operator++(int)
        {
            CircleTransformIterator res(*this);

            ++subdivision_i;
            angle += angle_step;

            return res;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different circles, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator==(const CircleTransformIterator &rhs) const
        {
            return subdivision_i == rhs.subdivision_i;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different circles, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        bool operator!=(const CircleTransformIterator &rhs) const
        {
            return subdivision_i != rhs.subdivision_i;
        }

        Aff_transformation_3 operator*() const
        {
            return TransformFunctor(angle, r);
        }

        int steps_left() const
        {
            return subdivision_i_end - subdivision_i;
        }
    };

    typedef CircleTransformIterator<Aff_transformation_3::rotate_x>   XCircleTransformIterator;
    typedef CircleTransformIterator<Aff_transformation_3::rotate_y>   YCircleTransformIterator;
    typedef CircleTransformIterator<Aff_transformation_3::rotate_z>   ZCircleTransformIterator;
}

#endif // _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H
