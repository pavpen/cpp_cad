#ifndef _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H

#include <iterator>

#include "../Aff_transformation_3.h"
#include "../reference_frame.h"


namespace cpp_cad
{
    template <Aff_transformation_3 TransformFunctor(double angle)>
    class RotateTransformIterator
    : public std::iterator<std::random_access_iterator_tag, Aff_transformation_3, int>
    {
        private:

        double angle;
        double angle_step;
        int subdivision_i;

        public:

        RotateTransformIterator(
            double start_angle = 0, double end_angle = 2 * M_PI,
            int subdivision_c = 16, int subdivision_i = 0)
        : angle(start_angle),
            angle_step((end_angle - start_angle) / subdivision_c),
            subdivision_i(subdivision_i)
        {}

        RotateTransformIterator(const RotateTransformIterator &source)
        : angle(source.angle),
            angle_step(source.angle_step),
            subdivision_i(source.subdivision_i)
        {}

        inline Aff_transformation_3 operator*() const
        {
            return TransformFunctor(angle);
        }

        inline Aff_transformation_3 operator[](const difference_type i) const
        {
            return TransformFunctor(angle + i * angle_step);
        }

        inline RotateTransformIterator &operator++()
        {
            ++subdivision_i;
            angle += angle_step;

            return *this;
        }

        inline RotateTransformIterator &operator--()
        {
            --subdivision_i;
            angle -= angle_step;

            return *this;
        }

        inline RotateTransformIterator operator++(int)
        {
            RotateTransformIterator res(*this);

            ++subdivision_i;
            angle += angle_step;

            return res;
        }

        inline RotateTransformIterator operator--(int)
        {
            RotateTransformIterator res(*this);

            --subdivision_i;
            angle += angle_step;

            return res;
        }

        inline difference_type operator-(const RotateTransformIterator &rhs) const
        {
            return subdivision_i - rhs.subdivision_i;
        }

        inline RotateTransformIterator operator+(const difference_type rhs) const
        {
            return RotateTransformIterator(
                start_angle, end_angle, subdivision_c, subdivision_i + rhs);
        }

        friend inline RotateTransformIterator operator+(
            const difference_type lhs, const RotateTransformIterator &rhs)
        {
            return rhs + lhs;
        }

        inline RotateTransformIterator &operator+=(const difference_type rhs)
        {
            subdivision_i += rhs;

            return *this;
        }

        inline RotateTransformIterator &operator-=(const difference_type rhs)
        {
            subdivision_i -= rhs;

            return *this;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        inline bool operator==(const RotateTransformIterator &rhs) const
        {
            return subdivision_i == rhs.subdivision_i;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different rotations, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        inline bool operator!=(const RotateTransformIterator &rhs) const
        {
            return subdivision_i != rhs.subdivision_i;
        }

        inline bool operator>(const RotateTransformIterator &rhs) const
        {
            return subdivision_i > rhs.subdivision_i;
        }

        inline bool operator>=(const RotateTransformIterator &rhs) const
        {
            return subdivision_i >= rhs.subdivision_i;
        }

        inline bool operator<(const RotateTransformIterator &rhs) const
        {
            return subdivision_i < rhs.subdivision_i;
        }

        inline bool operator<=(const RotateTransformIterator &rhs) const
        {
            return subdivision_i <= rhs.subdivision_i;
        }
    };

    typedef RotateTransformIterator<Aff_transformation_3::rotate_x>   XRotateTransformIterator;
    typedef RotateTransformIterator<Aff_transformation_3::rotate_y>   YRotateTransformIterator;
    typedef RotateTransformIterator<Aff_transformation_3::rotate_z>   ZRotateTransformIterator;
}

#endif // _CPP_CAD_ROTATE_TRANSFORM_ITERATOR_H
