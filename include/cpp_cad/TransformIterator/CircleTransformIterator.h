#ifndef _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H
#define _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H

#include <iterator>

#include "../Aff_transformation_3.h"
#include "../reference_frame.h"


namespace cpp_cad
{
    template <Aff_transformation_3 TransformFunctor(double angle, double r)>
    class CircleTransformIterator
    : public std::iterator<std::random_access_iterator_tag, Aff_transformation_3, int>
    {
        private:

        double r;
        double angle;
        double angle_step;
        int subdivision_i;

        public:

        CircleTransformIterator(
            double r, double start_angle = 0, double end_angle = 2 * M_PI,
            int subdivision_c = 16, int subdivision_i = 0)
        : r(r),
            angle(start_angle),
            angle_step((end_angle - start_angle) / subdivision_c),
            subdivision_i(subdivision_i)
        {}

        CircleTransformIterator(const CircleTransformIterator &source)
        : r(source.r),
            angle(source.angle),
            angle_step(source.angle_step),
            subdivision_i(source.subdivision_i)
        {}

        inline Aff_transformation_3 operator*() const
        {
            return TransformFunctor(angle, r);
        }

        inline Aff_transformation_3 operator[](const difference_type i) const
        {
            return TransformFunctor(angle + i * angle_step, r);
        }

        inline CircleTransformIterator &operator++()
        {
            ++subdivision_i;
            angle += angle_step;

            return *this;
        }

        inline CircleTransformIterator &operator--()
        {
            --subdivision_i;
            angle -= angle_step;

            return *this;
        }

        inline CircleTransformIterator operator++(int)
        {
            CircleTransformIterator res(*this);

            ++subdivision_i;
            angle += angle_step;

            return res;
        }

        inline CircleTransformIterator operator--(int)
        {
            CircleTransformIterator res(*this);

            --subdivision_i;
            angle -= angle_step;

            return res;
        }

        inline difference_type operator-(const CircleTransformIterator& rhs) const
        {
            return subdivision_i - rhs.subdivision_i;
        }

        inline CircleTransformIterator operator+(const difference_type rhs) const
        {
            return CircleTransformIterator(
                r, start_angle, end_angle, subdivision_c, subdivision_i + rhs);
        }

        inline CircleTransformIterator operator-(const difference_type rhs) const
        {
            return CircleTransformIterator(
                r, start_angle, end_angle, subdivision_c, subdivision_i - rhs);
        }

        friend inline CircleTransformIterator operator+(
            const difference_type lhs, const CircleTransformIterator &rhs)
        {
            return rhs + lhs;
        }

        inline CircleTransformIterator &operator+=(const difference_type rhs)
        {
            subdivision_i += rhs;

            return *this;
        }

        inline CircleTransformIterator &operator-=(const difference_type rhs)
        {
            subdivision_i -= rhs;

            return *this;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different circles, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        inline bool operator==(const CircleTransformIterator &rhs) const
        {
            return subdivision_i == rhs.subdivision_i;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different circles, or, otherwise, not derived from each other by
        // forward, or backward iteration.
        inline bool operator!=(const CircleTransformIterator &rhs) const
        {
            return subdivision_i != rhs.subdivision_i;
        }

        inline bool operator>(const CircleTransformIterator &rhs) const
        {
            return subdivision_i >= rhs.subdivision_i;
        }

        inline bool operator>=(const CircleTransformIterator &rhs) const
        {
            return subdivision_i >= rhs.subdivision_i;
        }

        inline bool operator<(const CircleTransformIterator &rhs) const
        {
            return subdivision_i < rhs.subdivision_i;
        }

        inline bool operator<=(const CircleTransformIterator &rhs) const
        {
            return subdivision_i <= rhs.subdivision_i;
        }
    };

    typedef CircleTransformIterator<Aff_transformation_3::rotate_x>   XCircleTransformIterator;
    typedef CircleTransformIterator<Aff_transformation_3::rotate_y>   YCircleTransformIterator;
    typedef CircleTransformIterator<Aff_transformation_3::rotate_z>   ZCircleTransformIterator;
}

#endif // _CPP_CAD_CIRCLE_TRANSFORM_ITERATOR_H
