#ifndef _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP
#define _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP

#include <iterator>

#include <operation_log.h>

#include "../Polygon_2.h"


namespace cpp_cad
{

// Constructs an iterator over the series of polygons given by a polygon and a
// transform iterator.
template <class TransformIterator>
class Polygon_2_TransformIterator
: public std::iterator<std::random_access_iterator_tag, Aff_transformation_3, int>
{
private:
    const Polygon_2 &polygon;
    TransformIterator transform_iterator;

public:
    Polygon_2_TransformIterator(
        TransformIterator transform_iterator, const Polygon_2 &polygon)
    : polygon(polygon),
        transform_iterator(transform_iterator)
    {}

    Polygon_2_TransformIterator(const Polygon_2_TransformIterator &source)
    : polygon(source.polygon),
        transform_iterator(source.transform_iterator)
    {}

    inline const Polygon_2 operator*() const
    {
        return transform(*transform_iterator, polygon);
    }

    inline Polygon_2 operator[](const difference_type i) const
    {
        return transform(transform_iterator[i], polygon);
    }

    inline Polygon_2_TransformIterator &operator++()
    {
        ++transform_iterator;

        return *this;
    }

    inline Polygon_2_TransformIterator &operator--()
    {
        --transform_iterator;

        return *this;
    }

    inline Polygon_2_TransformIterator &operator++(int)
    {
        Polygon_2_TransformIterator res(*this);

        ++transform_iterator;

        return res;
    }

    inline Polygon_2_TransformIterator &operator--(int)
    {
        Polygon_2_TransformIterator res(*this);

        --transform_iterator;

        return res;
    }

    inline difference_type operator-(const Polygon_2_TransformIterator& rhs) const
    {
        return transform_iterator - rhs.transform_iterator;
    }

    inline Polygon_2_TransformIterator operator+(const difference_type rhs) const
    {
        return Polygon_2_TransformIterator(transform_iterator + rhs, polygon);
    }

    inline Polygon_2_TransformIterator operator-(const difference_type rhs) const
    {
        return Polygon_2_TransformIterator(transform_iterator - rhs, polygon);
    }

    friend inline Polygon_2_TransformIterator operator+(
        const difference_type lhs, const Polygon_2_TransformIterator &rhs)
    {
        return rhs + lhs;
    }

    inline Polygon_2_TransformIterator &operator+=(const difference_type rhs)
    {
        transform_iterator += rhs;

        return *this;
    }

    inline Polygon_2_TransformIterator &operator-=(const difference_type rhs)
    {
        transform_iterator -= rhs;

        return *this;
    }

    inline bool operator==(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator == rhs.transform_iterator;
    }

    inline bool operator!=(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator != rhs.transform_iterator;
    }

    inline bool operator>(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator >= rhs.transform_iterator;
    }

    inline bool operator>=(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator >= rhs.transform_iterator;
    }

    inline bool operator<(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator < rhs.transform_iterator;
    }

    inline bool operator<=(const Polygon_2_TransformIterator &rhs) const
    {
        return transform_iterator <= rhs.transform_iterator;
    }
};

}

#endif // _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP
