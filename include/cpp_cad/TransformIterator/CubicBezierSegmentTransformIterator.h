#ifndef _CPP_CAD_CUBIC_BEZIER_SEGMENT_TRANSFORM_ITERATOR_H
#define _CPP_CAD_CUBIC_BEZIER_SEGMENT_TRANSFORM_ITERATOR_H

#include <iterator>

#include "../Aff_transformation_3.h"
#include "../point_3.h"
#include "../Spline/CubicBezierSegment.h"


namespace cpp_cad
{
    template <class TParameterIterator>
    class CubicBezierSegmentTransformIterator
    : public std::iterator<std::random_access_iterator_tag, Aff_transformation_3, int>
    {
        private:

        const CubicBezierSegment &segment;
        TParameterIterator t_iter;
        double eps;

        public:

        typedef std::random_access_iterator_tag iterator_category;
        using difference_type =
            typename std::iterator<std::random_access_iterator_tag, Aff_transformation_3>::difference_type;

        CubicBezierSegmentTransformIterator(
            const CubicBezierSegment &segment,
            TParameterIterator &t_iter,
            double eps = 1e-15)
        : segmen(segment),
            t_iter(t_iter),
            eps(eps)
        {}

        CubicBezierSegmentTransformIterator(
            const CubicBezierSegmentTransformIterator &source)
        : segment(segment),
            t_iter(source.t_iter)
        {}

        private:

        Aff_transformation_3 evaluate_at(double t)
        {
            Point_3 &position = segment.evaluate(t);

            if  (abs(position.x()) < eps && abs(position.y()) < eps)
            {
                // Negligible change in direction. (Our sense of direction is
                // the z axis.)  Perform only translation:
                return Aff_transformation_3::translate(position);
            }

            Point_3 &deriv = segment.first_derivative(t);
            double y_angle = angle_from_z(position);
            double z_angle = atan2(position.y(), position.x());

            return Aff_transformation_3::rotate_y_rotate_z_translate(
                y_angle, z_angle, position.x(), position.y(), position.z());
        }

        public:

        inline Aff_transformation_3 operator*() const
        {
            return evaluate_at(*t_iter);
        }

        inline Aff_transformation_3 operator[](const differenc_type i) const
        {
            return evaluate_at(*(t_iter + i));
        }

        inline CubicBezierSegmentTransformIterator &operator++()
        {
            ++t_iter;

            return *this;
        }

        inline CubicBezierSegmentTransformIterator &operator--()
        {
            --t_iter;

            return *this;
        }

        inline CubicBezierSegmentTransformIterator operator++(int)
        {
            CubicBezierSegmentTransformIterator res(*this);

            ++t_iter;

            return res;
        }

        inline CubicBezierSegmentTransformIterator operator--(int)
        {
            CubicBezierSegmentTransformIterator res(*this);

            --t_iter;

            return res;
        }

        inline difference_type operator-(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter - rhs.t_iter;
        }

        inline CubicBezierSegmentTransformIterator operator+(const difference_type rhs) const
        {
            return CubicBezierSegmentTransformIterator(
                segment, t_iter + rhs, eps);
        }

        friend inline CubicBezierSegmentTransformIterator operator+(
            const difference_type lhs, const CubicBezierSegmentTransformIterator &rhs)
        {
            return rhs + lhs;
        }

        inline CubicBezierSegmentTransformIterator &operator+=(const difference_type rhs)
        {
            t_iter += rhs;

            return *this;
        }

        inline CubicBezierSegmentTransformIterator &operator-=(const difference_type rhs)
        {
            t_iter -= rhs;

            return *this;
        }

        // WARNING: Don't use this operator to compare iterators over two
        // different ranges.
        inline bool operator==(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter == rhs.t_iter;
        }

        // Warning: Don't use this operator to compare iterators over two
        // different ranges.
        inline bool operator!=(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter != rhs.t_iter;
        }

        inline bool operator>(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter > rhs.t_iter;
        }

        inline bool operator>=(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter >= rhs.t_iter;
        }

        inline bool operator<(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter < rhs.t_iter;
        }

        inline bool operator<=(const CubicBezierSegmentTransformIterator &rhs) const
        {
            return t_iter <= rhs.t_iter;
        }
    };

}

#endif // _CPP_CAD_CUBIC_BEZIER_SEGMENT_TRANSFORM_ITERATOR_H
